import math
from simple_map import Map
from typing import Any, List, Dict, Optional
from dataclasses import dataclass


@dataclass
class Task:
    """Represents a task for the robot to perform."""
    target_id: str
    type: str
    options: Dict[str, str] = {}
    score: float = 0.0

class TaskScheduler:
    """Simple task scheduler that generates tasks based on the current map state."""
    must_dropoff_time = 12.0
    must_go_home_time = 7.0 # time at 1m distance from nest
    def __init__(self, map: Map, pathfinder_client=None, ros_node=None):
        self.map = map
        self.pathfinder = pathfinder_client
        self.task_queue: List[Task] = []
        self.tracked_task: Task = None
        self.tracked_task_invalid = False
        self._map_qos = 10

        # Explicit ROS wiring only:
        # - If a node is provided and map is a plain data-map, attach subscribers on it.
        # - If map is already a ROS node (e.g. MapNode), do not attach again.
        # - No internal node/executor is created by TaskScheduler.
        if ros_node is not None:
            self.attach_map_subscribers(ros_node, self._map_qos)

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
    
    def check_return_home_flag(self) -> bool:
        """Check if the return home flag is set."""
        return self.return_home_flag
    
    def track_task(self, task: Task) -> None:
        """Track the specified task."""
        self.tracked_task = task
        self.tracked_task_invalid = False

    def untrack_task(self) -> None:
        """Untrack the currently tracked task."""
        self.tracked_task = None
    
    def calculate_score(self, task: Task) -> float:
        # TODO: Implement a more sophisticated scoring mechanism based on the current map state, task type, and other heuristics.
        """Calculate a score for the given task based on the current map state."""
        # Placeholder scoring logic - can be expanded with more complex heuristics
        if task.type == "pickup":
            batch = self.map.batches.get(task.target_id)
            if batch:
                distance = ((self.map.robot_position.x - batch.x) ** 2 + (self.map.robot_position.y - batch.y) ** 2) ** 0.5
                return 1 / (distance + 1e-5)  # Higher score for closer batches
        elif task.type == "dropoff":
            dropoff = self.map.drop_offs.get(task.target_id)
            if dropoff:
                distance = ((self.map.robot_position.x - dropoff["x"]) ** 2 + (self.map.robot_position.y - dropoff["y"]) ** 2) ** 0.5
                return 1 / (distance + 1e-5)  # Higher score for closer dropoffs
        return 0.0
    
    def check_side(self, task: Task) -> None:
        task.options["side"] = "back"
        pass
    
    def predict_enemy_goal(self) -> None:
        """Predict the goal position of the enemy based on their current speed and position."""
        pass

    def generate_tasks(self) -> None:
        """Generate tasks based on the current map state."""
        #TODO: Implement task generation logic based on the current map state, such as creating pickup tasks for detected batches and dropoff tasks for known dropoff locations.
        queue = []
        if self.map.robot_front_empty or self.map.robot_back_empty:
            for batch_id, batch in self.map.batches.items():
                if batch.color != "unknown":
                    task = Task(target_id=batch_id, type="pickup")
                    task.score = self.calculate_score(task)
                    if batch_id in ["s3", "s5", "f3", "f5", "f8"]:
                        self.check_side(task)
                    queue.append(task)
        if not self.map.robot_front_empty or not self.map.robot_back_empty:
            for dropoff_id, dropoff in self.map.drop_offs.items():
                if dropoff["status"] == "empty":
                    task = Task(target_id=dropoff_id, type="dropoff")
                    task.score = self.calculate_score(task)
                    if dropoff_id in ["f3", "f5", "f8"]:
                        self.check_side(task)
                    queue.append(task)
        # sorting tasks
        queue.sort(key=lambda t: t.score, reverse=True)
        if self.map.time_left < self.must_dropoff_time:
            # If time is running out, prioritize dropoff tasks
            queue = [t for t in queue if t.type == "dropoff"] + [t for t in queue if t.type != "dropoff"]
        if (self.map.time_left < max(self.must_go_home_time * self.map.distance_from(self.map.robot_position, self.map.nest_position), 5)) and len([t for t in queue if t.type == "dropoff"]) == 0:
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
        #TODO: Implement logic to check if the currently tracked task is still valid based on the current map state. Return a string indicating the reason for invalidity if it is no longer valid, or None if it is still valid.
        self.tracked_task_invalid = False

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
    
    def loop(self) -> None:
        """Main loop to update tasks based on the current map state."""
        if hasattr(self.map, 'sync_time_left_from_round'):
            self.map.sync_time_left_from_round()
        self.generate_tasks()
        self.check_tracked_task_validity()
