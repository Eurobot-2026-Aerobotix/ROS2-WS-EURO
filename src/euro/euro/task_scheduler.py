import math
import re
try:
    from .simple_map import Map, Pose2D
except ImportError:
    # Fallback for direct execution in development contexts.
    from simple_map import Map, Pose2D
from typing import Any, List, Dict, Optional
from dataclasses import dataclass, field


@dataclass
class Task:
    """Represents a task for the robot to perform."""
    target_id: str
    type: str
    options: Dict[str, str] = field(default_factory=dict)
    score: float = 0.0
    # Runtime metadata for execution tracking
    end_action: str = ""          # '', 'pick_up', or 'put_down'
    batch_known_id: str = ""      # e.g. 's3', 'f5'
    task_name: str = ""           # source task name from strategy list

class TaskScheduler:
    """Simple task scheduler that generates tasks based on the current map state."""
    must_dropoff_time = 1200.0
    must_go_home_time = 7000.0 # time at 1m distance from nest
    max_distance_for_predict_enemy_goal = 1500.0 # in meters
    drop_off_distance = 300.0
    distance_from_wall = 200.0
    coeffs = {
        "is_enemy_goal": 0.001,
        "distance_to_goal": -0.001,
        "number_of_obstacles_between": -0.5,
        "distance_to_enemy": 0.003,
        "number_of_opposite_crates": 1
    }
    def __init__(self, map: Map, pathfinder_client=None, ros_node=None, loop_period_s: float = 0.05):
        self.map = map
        self.pathfinder = pathfinder_client
        self.task_queue: List[Task] = []
        self.rpi_queue: List[Dict[str, Any]] = []
        self.tracked_task: Task = None
        self.tracked_task_invalid = False
        self._map_qos = 10
        self._loop_period_s = float(loop_period_s)
        self._loop_timer = None

        # Explicit ROS wiring only:
        # - If a node is provided and map is a plain data-map, attach subscribers on it.
        # - If map is already a ROS node (e.g. MapNode), do not attach again.
        # - No internal node/executor is created by TaskScheduler.
        if ros_node is not None:
            self.attach_map_subscribers(ros_node, self._map_qos)
            self.attach_loop_timer(ros_node, self._loop_period_s)

    def attach_loop_timer(self, ros_node, period_s: float = 0.05) -> None:
        """Attach periodic scheduler `loop()` timer on provided ROS node."""
        if self._loop_timer is not None:
            return
        self._loop_period_s = max(0.01, float(period_s))
        self._loop_timer = ros_node.create_timer(self._loop_period_s, self.loop)

    def attach_map_subscribers(self, ros_node, qos: int = 10) -> None:
        """
        Attach simple_map topic subscribers using the provided ROS node.

        Compatibility behavior:
        - Plain `Map` + external node: subscribers are attached here.
        - `MapNode`: subscribers are already attached in map-node init, so this is a no-op.
        """
        if hasattr(self.map, 'attach_ros_subscribers'):
            should_attach_subs = True

            # If map itself is the provided node (MapNode pattern), avoid duplicate subscriptions.
            if ros_node is self.map:
                should_attach_subs = False

            # If subscriptions already exist, assume map has already been wired.
            existing_subs = getattr(self.map, '_subscriptions', None)
            if isinstance(existing_subs, list) and len(existing_subs) > 0:
                should_attach_subs = False

            if should_attach_subs:
                self.map.attach_ros_subscribers(ros_node, qos=qos)

        # Attach map timer update during active round window [5s, 100s].
        if hasattr(self.map, 'attach_round_update_timer'):
            self.map.attach_round_update_timer(
                ros_node,
                period_s=1.0,
                start_s=5.0,
                end_s=100.0,
                pathfinder_client=self.pathfinder,
            )

        self.predicted_enemy_goal : str = "unknown"

    def pop_task(self) -> Optional[Task]:
        """Pop the next task from the queue."""
        if self.task_queue:
            task = self.task_queue.pop(0)
            self.track_task(task)
            return task
        return None
    
    def track_task(self, task: Task) -> None:
        """Track the specified task."""
        self.tracked_task = task
        self.tracked_task_invalid = False

    @staticmethod
    def _infer_end_action_from_sequence(sequence: List[Dict[str, Any]]) -> str:
        """Infer terminal actuator action from task sequence."""
        if not sequence:
            return ""
        for cmd in reversed(sequence):
            command = str(cmd.get('command', '')).strip().lower()
            if command in ('pick_up', 'put_down'):
                return command
        return ""

    @staticmethod
    def _infer_batch_known_id_from_name(task_name: str) -> str:
        """
        Best-effort extraction of batch known_id from task name.

        Supported prefixes:
        - pickup_<known_id>_...
        - putdown_<known_id>_...
        - putdown_insert_<known_id>_...
        - steal_<known_id>_...
        """
        if not task_name:
            return ""

        name = str(task_name).strip()
        patterns = [
            r'^pickup_([^_]+)_',
            r'^putdown_insert_([^_]+)_',
            r'^putdown_([^_]+)_',
            r'^steal_([^_]+)_',
        ]
        for pattern in patterns:
            m = re.match(pattern, name, flags=re.IGNORECASE)
            if m:
                return str(m.group(1)).lower()
        return ""

    def build_tracking_task_from_runtime_dict(self, runtime_task: Dict[str, Any]) -> Task:
        """Build a `Task` instance from strategy runtime task dict for tracking."""
        task_name = str(runtime_task.get('name', '') or '')
        sequence = runtime_task.get('sequence', []) or []
        end_action = self._infer_end_action_from_sequence(sequence)
        batch_known_id = self._infer_batch_known_id_from_name(task_name)

        inferred_type = 'runtime'
        if end_action == 'pick_up':
            inferred_type = 'pickup'
        elif end_action == 'put_down':
            inferred_type = 'dropoff'

        target_id = batch_known_id if batch_known_id else task_name

        return Task(
            target_id=target_id,
            type=inferred_type,
            options={},
            score=0.0,
            end_action=end_action,
            batch_known_id=batch_known_id,
            task_name=task_name,
        )

    def track_runtime_task_dict(self, runtime_task: Dict[str, Any]) -> Task:
        """Infer and track runtime task execution metadata from strategy-manager task dict."""
        task = self.build_tracking_task_from_runtime_dict(runtime_task)
        self.track_task(task)
        return task

    def untrack_task(self) -> None:
        """Untrack the currently tracked task."""
        self.tracked_task = None
    
    def calculate_score(self, task: Task) -> float:
        """Calculate a score for the given task based on the current map state."""
        # Placeholder scoring logic - can be expanded with more complex heuristics
        if task.type == "pickup":
            batch = self.map.batches.get(task.target_id)
            if batch is None:
                return float("-inf")
            target_pos = Pose2D(x=batch.x, y=batch.y)
        elif task.type == "dropoff":
            dropoff = self.map.drop_offs.get(task.target_id)
            if dropoff is None:
                return float("-inf")
            target_pos = Pose2D(x=float(dropoff["x"]), y=float(dropoff["y"]))
        else:
            return 0.0

        score = 0.0
        if self.predicted_enemy_goal != "unknown" and task.target_id == self.predicted_enemy_goal:
            score += self.coeffs["is_enemy_goal"] * (self.map.distance_from(self.map.robot_position, target_pos) - self.map.distance_from(self.map.enemy_absolute, target_pos))
        else:
            score += self.coeffs["is_enemy_goal"] * 700.0
        score += self.coeffs["distance_to_goal"] * self.map.distance_from(self.map.robot_position, target_pos)
        score += self.map.calculate_number_of_obstacles_to(target_pos) * self.coeffs["number_of_obstacles_between"]
        if self.map.enemy_relative.detected:
            score += self.coeffs["distance_to_enemy"] * self.map.distance_from(self.map.enemy_absolute, target_pos)
        else:
            score += self.coeffs["distance_to_enemy"] * 1000.0  # If enemy not detected, assume far away
        if task.type == "pickup":
            batch = self.map.batches.get(task.target_id)
            if batch is not None:
                pattern = str(batch.color or "")
                score += self.coeffs["number_of_opposite_crates"] * (len(pattern) - pattern.count(self.map.robot_color[0]))
        return score
    
    def choose_side(self, task: Task) -> bool:
        if task.type == "pickup":
            pickup_pos = []
            if task.target_id in ["s1", "f1", "s2"]:
                pickup_pos.append(Pose2D(x=self.map.batches[task.target_id].x, y=self.map.batches[task.target_id].y + 175.0))
            elif task.target_id in ["f2", "s4", "f6", "s6", "f9"]:
                pickup_pos.append(Pose2D(x=self.map.batches[task.target_id].x - 175.0, y=self.map.batches[task.target_id].y))
            elif task.target_id in ["s7", "f10", "s8"]:
                pickup_pos.append(Pose2D(x=self.map.batches[task.target_id].x, y=self.map.batches[task.target_id].y - 175.0))
            elif task.target_id in ["f4", "f7"]:
                pickup_pos.append(Pose2D(x=self.map.batches[task.target_id].x + 175.0, y=self.map.batches[task.target_id].y))
            elif task.target_id in ["s3", "f5"]:
                pickup_pos.append(Pose2D(x=self.map.batches[task.target_id].x - 175.0, y=self.map.batches[task.target_id].y))
                pickup_pos.append(Pose2D(x=self.map.batches[task.target_id].x + 175.0, y=self.map.batches[task.target_id].y))
            for pos in pickup_pos:
                if self.map.distance_from(self.map.enemy_absolute, pos) < self.map.enemy_radius + self.map.robot_radius + 50.0:
                    pickup_pos.remove(pos)
            pickup_pos.sort(key=lambda p: self.map.distance_from(self.map.robot_position, p))
            if len(pickup_pos) == 0:
                return False
            task.options["x"] = pickup_pos[0].x
            task.options["y"] = pickup_pos[0].y
        elif task.type == "dropoff":
            dropoff = self.map.drop_offs.get(task.target_id)
            if dropoff is None:
                return False

            center_x = float(dropoff["x"])
            center_y = float(dropoff["y"])
            radius = float(self.drop_off_distance)
            sample_count = 72

            best_pos: Optional[Pose2D] = None
            best_dist = float("inf")

            for i in range(sample_count):
                angle = (2.0 * math.pi * float(i)) / float(sample_count)
                candidate = Pose2D(
                    x=center_x + radius * math.cos(angle),
                    y=center_y + radius * math.sin(angle),
                )
                if (task.target_id in ["f1"] and (candidate.y < self.distance_from_wall)):
                    continue
                if (task.target_id in ["f10"] and (candidate.y > 3000.0 - self.distance_from_wall)):
                    continue
                if (task.target_id in ["f2", "f6", "f9"] and (candidate.x > 2000.0 - self.distance_from_wall)):
                    continue
                if (task.target_id in ["f4", "f7"] and (candidate.x < 450.0 + self.distance_from_wall)):
                    continue

                if self.map.check_collision(candidate, ignore_batch_id=task.target_id):
                    continue

                dist = self.map.distance_from(self.map.robot_position, candidate)
                if dist < best_dist:
                    best_dist = dist
                    best_pos = candidate

            if best_pos is None:
                return False

            task.options["x"] = best_pos.x
            task.options["y"] = best_pos.y
        return True
    
    def predict_enemy_goal(self) -> None:
        """Predict the goal position of the enemy based on their current speed and position."""
        self.predicted_enemy_goal = "unknown"

        if self.map.enemy_relative.detected == False:
            return

        enemy_pos = self.map.enemy_absolute
        enemy_speed = self.map.enemy_speed

        speed_mag = math.hypot(enemy_speed.x, enemy_speed.y)
        if speed_mag < 1e-3:
            return

        angle_tolerance_deg = 30.0

        # Prefer currently tracked pickups when available, otherwise fallback to predefined pickups.
        pickup_targets: Dict[str, Dict[str, float]] = {}
        for batch_id, batch in self.map.predefined_batch_positions.items():
            pickup_targets[batch_id] = {"x": float(batch["x"]), "y": float(batch["y"])}

        # Build candidate list: all pickups + all drop-offs.
        candidates: List[Dict[str, float]] = []
        for target_id, target in pickup_targets.items():
            candidates.append({"id": target_id, "x": target["x"], "y": target["y"]})
        for target_id, target in self.map.drop_offs.items():
            candidates.append({"id": target_id, "x": float(target["x"]), "y": float(target["y"])})

        best_id = "unknown"
        best_distance = float("inf")

        for target in candidates:
            to_target_x = target["x"] - enemy_pos.x
            to_target_y = target["y"] - enemy_pos.y
            distance = math.hypot(to_target_x, to_target_y)
            if distance < 1e-3:
                continue

            dot = (enemy_speed.x * to_target_x) + (enemy_speed.y * to_target_y)
            cos_angle = dot / (speed_mag * distance)
            cos_angle = max(-1.0, min(1.0, cos_angle))
            angle_deg = math.degrees(math.acos(cos_angle))

            if abs(angle_deg) <= angle_tolerance_deg and distance < best_distance and distance < self.max_distance_for_predict_enemy_goal:
                best_distance = distance
                best_id = str(target["id"])

        self.predicted_enemy_goal = best_id

    def generate_tasks(self) -> None:
        """Generate tasks based on the current map state."""
        self.predict_enemy_goal()
        queue = []
        # generate pickup tasks if we have space in the robot
        if self.map.robot_front_empty or self.map.robot_back_empty:
            for batch_id, batch in self.map.batches.items():
                if batch_id in ["f3", "f5", "f8"]:
                    continue
                if (len(batch.color) != 4) or (batch.color.count(self.map.robot_color[0]) == 4):
                    continue
                if batch_id[0] == "f":
                    possible_pickup_phi = 0.0
                    if batch_id in ["f1", "f10"]:
                        possible_pickup_phi = 90.0
                    if batch.phi_degrees is None or abs(batch.phi_degrees - possible_pickup_phi) > 1.0:
                        continue
                task = Task(target_id=batch_id, type="pickup")
                task.score = self.calculate_score(task)
                if (self.choose_side(task)):
                    queue.append(task)
        
        # generate dropoff tasks if we have crates in the robot
        if not self.map.robot_front_empty or not self.map.robot_back_empty:
            for dropoff_id, dropoff in self.map.drop_offs.items():
                if dropoff_id in ["f3", "f5", "f8"] or dropoff["status"] != "empty":
                    continue
                task = Task(target_id=dropoff_id, type="dropoff")
                task.score = self.calculate_score(task)
                if (self.choose_side(task)):
                    queue.append(task)
        
        if self.tracked_task is not None:
            queue = [t for t in queue if t.target_id != self.tracked_task.target_id]

        if self.map.ninja_on_map:
            blocked_id = "f7" if self.map.robot_color == "blue" else "f4"
            queue = [t for t in queue if t.target_id != blocked_id]

        # sorting tasks
        queue.sort(key=lambda t: t.score, reverse=True)

        queue = [t for t in queue if t.target_id[0] == "s"] + [t for t in queue if t.target_id[0] == "f"]

        if self.map.get_time_left() < self.must_dropoff_time:
            # If time is running out, prioritize dropoff tasks
            queue = [t for t in queue if t.type == "dropoff"] + [t for t in queue if t.type != "dropoff"]
        
        if (self.map.get_time_left() < max(self.must_go_home_time * self.map.distance_from(self.map.robot_position, self.map.nest_position)/1000.0, 5)) and len([t for t in queue if t.type == "dropoff"]) == 0:
            task = Task(target_id="nest", type="return_home")
            task.options["drop"] = "none"
            if not self.map.robot_front_empty:
                task.options["drop"] = "front"
            elif not self.map.robot_back_empty:
                task.options["drop"] = "back"
            queue.insert(0, task)  # Prioritize return home task
            
        self.task_queue = queue
    
    def check_tracked_task_validity(self) -> Optional[str]:
        """Check the validity of the currently tracked task."""
        if self.tracked_task is None:
            return

        if self.tracked_task.type == "pickup":
            batch = self.map.batches.get(self.tracked_task.target_id)
            if batch is None:
                self.tracked_task_invalid = True
                return
            if (len(batch.color) != 4) or (batch.color.count(self.map.robot_color[0]) == 4):
                self.tracked_task_invalid = True
                return
        elif self.tracked_task.type == "dropoff":
            dropoff = self.map.drop_offs.get(self.tracked_task.target_id)
            if dropoff is None:
                self.tracked_task_invalid = True
                return
            if dropoff["status"] != "empty" and dropoff["type"] != "center":
                self.tracked_task_invalid = True
                return

    def cancel_tracked_task(self) -> bool:
        return self.tracked_task_invalid
    
    def build_pickup_task(
        self,
        batch_known_id: str,
        pickup_face: str,
        pose_selector: int,
        start_pose: Optional[Dict[str, float]] = None,
        approach_offset_mm: float = 100.0,
    ) -> Optional[Dict[str, Any]]:
        """
        Build a pickup task for a specific batch with explicit face and pose selection.
        Includes path planning from current or provided start pose.
        
        Args:
            batch_known_id: Batch ID key from pickup_poses (e.g., 'S1', 'S3')
            pickup_face: 'front' or 'back' for ESP device selection
            pose_selector: 0 for lowest X pickup pose, 1 for highest X pickup pose
            start_pose: Optional start pose dict {'x', 'y', 'theta'}
                        x/y in mm, theta in degrees
            approach_offset_mm: Distance before pickup pose for path target (default 100mm)
        
        Returns:
            Task dict compatible with strategy_manager queue format, or None if invalid inputs
        """
        # Pathfinder is required for path-based task generation
        if self.pathfinder is None:
            return None

        normalized_id = batch_known_id.upper()

        # Validate batch exists in pickup_poses
        if normalized_id not in self.map.pickup_poses:
            return None
        
        # Validate pickup_face
        if pickup_face not in ['front', 'back']:
            return None
        
        # Get poses for this batch
        poses = self.map.pickup_poses[normalized_id]
        if not poses:
            return None
        
        # Validate pose_selector
        if pose_selector not in [0, 1]:
            return None
        
        # If only one pose, use it regardless of selector
        if len(poses) == 1:
            selected_pose = poses[0]
        else:
            # Sort by x coordinate to determine lowest (0) vs highest (1)
            sorted_poses = sorted(poses, key=lambda p: p['x'])
            if pose_selector >= len(sorted_poses):
                return None
            selected_pose = sorted_poses[pose_selector]
        
        # Extract position and theta from selected pose
        pickup_x = selected_pose['x']  # in mm
        pickup_y = selected_pose['y']  # in mm
        pickup_thetas = selected_pose['theta']  # list of angles in degrees
        
        # Choose first theta from the list (or single value)
        if isinstance(pickup_thetas, list) and len(pickup_thetas) > 0:
            pickup_theta = pickup_thetas[0]
        else:
            pickup_theta = pickup_thetas if isinstance(pickup_thetas, (int, float)) else 0
        
        # Determine ESP device and STM command type
        esp_device = 'ESP_front' if pickup_face == 'front' else 'ESP_back'
        stm_command = 'go_to' if pickup_face == 'front' else 'go_to_backwards'

        # Pre-approach point: plan to a point before final pickup pose, then do final precise move.
        # Uses pickup orientation to compute a point behind the pickup pose.
        offset = max(0.0, float(approach_offset_mm))
        theta_rad = math.radians(float(pickup_theta))
        approach_x = pickup_x - (offset * math.cos(theta_rad))
        approach_y = pickup_y - (offset * math.sin(theta_rad))

        # Plan path to pre-approach target
        if start_pose is not None:
            if not all(k in start_pose for k in ('x', 'y', 'theta')):
                return None
            stm32_commands, path_success = self.pathfinder.path_full(
                target_x=approach_x / 1000.0,
                target_y=approach_y / 1000.0,
                target_theta=theta_rad,
                start_x=float(start_pose['x']) / 1000.0,
                start_y=float(start_pose['y']) / 1000.0,
                start_theta=math.radians(float(start_pose['theta']))
            )
        else:
            stm32_commands, path_success = self.pathfinder.path_full(
                target_x=approach_x / 1000.0,
                target_y=approach_y / 1000.0,
                target_theta=theta_rad,
            )

        if not path_success:
            return None
        
        # Build task sequence compatible with strategy_executor
        task_sequence: List[Dict[str, Any]] = []

        # Add waypoints from pathfinder
        last_cmd = None
        for cmd in stm32_commands:
            values = cmd.replace('go_to(', '').replace(')', '').split(',')
            x = float(values[0].strip())
            y = float(values[1].strip())
            if cmd != last_cmd:
                task_sequence.append({
                    'device': 'STM',
                    'command': stm_command,
                    'params': [x, y, 1100.0,0.0, {'from_pathfinding': True, 'from_pickup_task': True}],
                    'wait': False
                })
            last_cmd = cmd
        task_sequence.append({
            'device': 'STM',
            'command': stm_command,
            'params': [pickup_x, pickup_y, 1100.0, 1.0, {'from_pathfinding': True, 'from_pickup_task': True}],
            'wait': False
        })
        task_sequence.append({
            'device': esp_device,
            'command': 'pick_up',
            'params': {},
            'wait': True
        })
        
        # Build and return task dict
        task = {
            'name': f'pickup_{normalized_id}_{pickup_face}',
            'sequence': task_sequence,
            'global_goal': {
                'x': pickup_x,
                'y': pickup_y,
                'theta': pickup_theta
            }
        }
        return task

    def build_put_down(
        self,
        batch_known_id: str,
        x: float,
        y: float,
        device: str,
        start_pose: Optional[Dict[str, float]] = None,
        insert_distance_mm: float = 200.0,
    ) -> Optional[List[Dict[str, Any]]]:
        """
        Build a put_down task.

        Flow:
        1) Pathfind to (x, y)
        2) Orientate toward target batch center from map.drop_offs
          3) Execute ESP put_down
          4) If type B, create a separate follow-up task with 20cm insertion move
              (front=go_to, back=go_to_backwards), preserving orientation

        Args:
            batch_known_id: Putdown zone batch id (e.g., 'f1', 'f5')
            x: Target x in mm
            y: Target y in mm
            device: 'front'/'back' or 'ESP_front'/'ESP_back'
            start_pose: Optional {'x','y','theta'} in mm/degrees for explicit path start
            insert_distance_mm: Insertion move distance for type B zones (default 200mm)

        Returns:
            Ordered list of task dicts compatible with strategy_manager queue format:
            - Type A: [putdown_task]
            - Type B: [putdown_task, putdown_insert_task]
            None on invalid inputs
        """
        if self.pathfinder is None:
            return None

        normalized_id = str(batch_known_id).lower()
        zone = self.map.drop_offs.get(normalized_id)
        if zone is None:
            return None

        device_key = str(device).strip().lower()
        if device_key in ('esp_front', 'front'):
            esp_device = 'ESP_front'
            stm_command = 'go_to'
        elif device_key in ('esp_back', 'back'):
            esp_device = 'ESP_back'
            stm_command = 'go_to_backwards'
        else:
            return None

        target_x = float(x)
        target_y = float(y)
        center_x = float(zone['x'])
        center_y = float(zone['y'])
        zone_type = str(zone.get('batch_type', 'A')).upper()

        # Angle at target point to face center of putdown zone
        theta_to_center_deg = math.degrees(math.atan2(center_y - target_y, center_x - target_x))
        theta_to_center_rad = math.radians(theta_to_center_deg)

        # Path to target point only; explicit orientate is added in task sequence
        current_theta_rad = math.radians(float(self.map.robot_position.theta))

        if start_pose is not None:
            if not all(k in start_pose for k in ('x', 'y', 'theta')):
                return None
            stm32_commands, path_success = self.pathfinder.path_full(
                target_x=target_x / 1000.0,
                target_y=target_y / 1000.0,
                target_theta=current_theta_rad,
                start_x=float(start_pose['x']) / 1000.0,
                start_y=float(start_pose['y']) / 1000.0,
                start_theta=math.radians(float(start_pose['theta']))
            )
        else:
            stm32_commands, path_success = self.pathfinder.path_full(
                target_x=target_x / 1000.0,
                target_y=target_y / 1000.0,
                target_theta=current_theta_rad,
            )

        if not path_success:
            return None

        task_sequence: List[Dict[str, Any]] = []

        # Waypoints from pathfinder
        last_cmd = None
        for cmd in stm32_commands:
            values = cmd.replace('go_to(', '').replace(')', '').split(',')
            waypoint_x = float(values[0].strip())
            waypoint_y = float(values[1].strip())
            if cmd != last_cmd:
                task_sequence.append({
                    'device': 'STM',
                    'command': stm_command,
                    'params': [waypoint_x, waypoint_y, 1100.0, 0.0, {'from_pathfinding': True, 'from_putdown_task': True}],
                    'wait': False
                })
            last_cmd = cmd

        # Final precise target + orientate to center
        task_sequence.append({
            'device': 'STM',
            'command': stm_command,
            'params': [target_x, target_y, 1100.0, 1.0, {'from_pathfinding': True, 'from_putdown_task': True}],
            'wait': False
        })
        task_sequence.append({
            'device': 'STM',
            'command': 'orientate',
            'params': [theta_to_center_deg, 0.0],
            'wait': False
        })

        task_sequence.append({
            'device': esp_device,
            'command': 'put_down',
            'params': {},
            'wait': True
        })

        putdown_task = {
            'name': f'putdown_{normalized_id}_{esp_device.split("_")[1]}',
            'sequence': task_sequence,
            'global_goal': {
                'x': target_x,
                'y': target_y,
                'theta': theta_to_center_deg
            }
        }

        # For type B, build insertion as a separate task executed after put_down
        if zone_type != 'B':
            return [putdown_task]

        dist = max(0.0, float(insert_distance_mm))
        # Always insert toward the center, regardless of front/back device.
        insert_x = target_x + dist * math.cos(theta_to_center_rad)
        insert_y = target_y + dist * math.sin(theta_to_center_rad)

        putdown_insert_task = {
            'name': f'putdown_insert_{normalized_id}_{esp_device.split("_")[1]}',
            'sequence': [
                {
                    'device': 'STM',
                    'command': stm_command,
                    'params': [insert_x, insert_y, 900.0, 1.0, {'from_putdown_task': True, 'type_b_insert': True}],
                    'wait': True
                }
            ],
            'global_goal': {
                'x': insert_x,
                'y': insert_y,
                'theta': theta_to_center_deg
            }
        }

        return [putdown_task, putdown_insert_task]

    def _select_pickup_face(self) -> Optional[str]:
        """Choose pickup face based on current holder availability."""
        if self.map.robot_front_empty:
            return 'front'
        if self.map.robot_back_empty:
            return 'back'
        return None

    def _select_pickup_pose_selector(self, task: Task) -> int:
        """Choose pickup pose selector (0/1) using task options or robot proximity."""
        normalized_id = str(task.target_id).upper()
        poses = self.map.pickup_poses.get(normalized_id, [])
        if len(poses) <= 1:
            return 0

        # If task already contains chosen point (from choose_side), map it to nearest pose.
        if 'x' in task.options and 'y' in task.options:
            tx = float(task.options['x'])
            ty = float(task.options['y'])
        else:
            tx = float(self.map.robot_position.x)
            ty = float(self.map.robot_position.y)

        sorted_poses = sorted(poses, key=lambda p: p['x'])

        def _d2(p):
            return (float(p['x']) - tx) ** 2 + (float(p['y']) - ty) ** 2

        best_idx = min(range(len(sorted_poses)), key=lambda i: _d2(sorted_poses[i]))
        return 0 if best_idx == 0 else 1

    def _select_putdown_device(self, task: Task) -> Optional[str]:
        """Choose putdown device based on holder occupancy (front/back)."""
        drop_pref = str(task.options.get('drop', '')).lower()
        if drop_pref in ('front', 'back'):
            return drop_pref

        # Prefer dropping what is currently held.
        if not self.map.robot_front_empty:
            return 'front'
        if not self.map.robot_back_empty:
            return 'back'
        return None

    def get_optimal_task(self, start_pose: Optional[Dict[str, float]] = None) -> Optional[List[Dict[str, Any]]]:
        """
        Pop the best scored high-level task, build executable task(s), and enqueue them in `rpi_queue`.

        Returns:
            List of executable task dict(s) that were enqueued, or None if no valid task.
        """
        task = self.pop_task()
        if task is None:
            return None

        generated_tasks: Optional[List[Dict[str, Any]]] = None

        if task.type == 'pickup':
            pickup_face = self._select_pickup_face()
            if pickup_face is None:
                self.tracked_task_invalid = True
                return None

            pose_selector = self._select_pickup_pose_selector(task)
            pickup_task = self.build_pickup_task(
                batch_known_id=str(task.target_id),
                pickup_face=pickup_face,
                pose_selector=pose_selector,
                start_pose=start_pose,
            )
            if pickup_task is not None:
                generated_tasks = [pickup_task]

        elif task.type == 'dropoff':
            device = self._select_putdown_device(task)
            if device is None:
                self.tracked_task_invalid = True
                return None

            zone = self.map.drop_offs.get(str(task.target_id).lower())
            if zone is None:
                self.tracked_task_invalid = True
                return None

            target_x = float(task.options.get('x', zone['x']))
            target_y = float(task.options.get('y', zone['y']))

            generated_tasks = self.build_put_down(
                batch_known_id=str(task.target_id),
                x=target_x,
                y=target_y,
                device=device,
                start_pose=start_pose,
            )

        else:
            # Only pickup/dropoff are converted here.
            return None

        if not generated_tasks:
            self.tracked_task_invalid = True
            return None

        self.rpi_queue.extend(generated_tasks)
        return generated_tasks
    
    def loop(self) -> None:
        """Main loop to update tasks based on the current map state."""
        if hasattr(self.map, 'sync_time_left_from_round'):
            self.map.sync_time_left_from_round()
        self.generate_tasks()
        self.check_tracked_task_validity()
