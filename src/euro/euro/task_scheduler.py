import math
import re
try:
    from .simple_map import Map, Pose2D
except ImportError:
    # Fallback for direct execution in development contexts.
    from simple_map import Map, Pose2D
from typing import Any, List, Dict, Optional, Tuple
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
    # NOTE: `Map.get_time_left()` is expressed in seconds.
    # Keep these thresholds in seconds as well.
    allow_fx = False
    must_dropoff_time = 5.0
    must_go_home_time = 25.0  # travel budget at 1m distance from nest (seconds)
    max_distance_for_predict_enemy_goal = 1500.0 # in meters
    drop_off_distance = 400.0
    retreat_distance = 250.0
    distance_from_wall = 200.0
    dropoff_min_clearance_to_sx = 420.0
    dropoff_min_clearance_to_fx = 330.0
    putdown_insert_distance = 175.0
    putdown_retreat_distance = 250.0
    switch_distance=550.0
    retreat_distance=175.0
    coeffs = {
        "is_enemy_goal": 0.0,
        "distance_to_goal_far": 0.001,
        "number_of_obstacles_between": -0.5,
        "distance_to_enemy": 0.001,
        "number_of_opposite_crates": 1,
        "distance_to_goal_close":0.008,
    }
    log_score_breakdown = True
    log_task_queue_snapshot = True
    def __init__(self, map: Map, pathfinder_client=None, ros_node=None, loop_period_s: float = 0.05):
        self.map = map
        self.pathfinder = pathfinder_client
        self.ros_node = ros_node
        self.task_queue: List[Task] = []
        self.rpi_queue: List[Dict[str, Any]] = []
        self.tracked_task: Task = None
        self.tracked_task_invalid = False
        self._map_qos = 10
        self._loop_period_s = float(loop_period_s)
        self._loop_timer = None
        self._last_available_batches_signature = None
        self._last_pickup_queue_signature = None
        self._last_task_queue_signature = None

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
        score, _ = self._calculate_score_with_breakdown(task)
        return score

    def _calculate_score_with_breakdown(self, task: Task) -> Tuple[float, Dict[str, float]]:
        """Calculate score and return per-term contributions for debugging/visualization."""
        if task.type == "pickup":
            batch = self.map.batches.get(task.target_id)
            if batch is None:
                return float("-inf"), {}
            target_pos = Pose2D(x=batch.x, y=batch.y)
        elif task.type == "dropoff":
            dropoff = self.map.drop_offs.get(task.target_id)
            if dropoff is None:
                return float("-inf"), {}
            target_pos = Pose2D(x=float(dropoff["x"]), y=float(dropoff["y"]))
        else:
            return 0.0, {}

        score = 0.0
        contributions: Dict[str, float] = {
            "is_enemy_goal": 0.0,
            "distance_to_goal": 0.0,
            "number_of_obstacles_between": 0.0,
            "distance_to_enemy": 0.0,
            "number_of_opposite_crates": 0.0,
        }

        if self.predicted_enemy_goal != "unknown" and task.target_id == self.predicted_enemy_goal:
            contributions["is_enemy_goal"] = self.coeffs["is_enemy_goal"] * (
                self.map.distance_from(self.map.robot_position, target_pos)
                - self.map.distance_from(self.map.enemy_absolute, target_pos)
            )
        else:
            contributions["is_enemy_goal"] = self.coeffs["is_enemy_goal"] * 700.0
        distance=self.map.distance_from(self.map.robot_position, target_pos)
        if distance < self.switch_distance:
            contributions["distance_to_goal"] = (
                self.coeffs["distance_to_goal_close"] * (self.switch_distance*2 - distance)
            )
        else:
            contributions["distance_to_goal"] = (
                self.coeffs["distance_to_goal_far"] * distance
            )
        contributions["number_of_obstacles_between"] = (
            self.map.calculate_number_of_obstacles_to(target_pos) * self.coeffs["number_of_obstacles_between"]
        )

        if self.map.enemy_relative.detected:
            contributions["distance_to_enemy"] = (
                self.coeffs["distance_to_enemy"] * self.map.distance_from(self.map.enemy_absolute, target_pos)
            )
        else:
            contributions["distance_to_enemy"] = self.coeffs["distance_to_enemy"] * 1000.0  # If enemy not detected, assume far away

        if task.type == "pickup":
            if task.target_id[0] == "f":
                batch = self.map.batches.get(task.target_id)
                if batch is not None:
                    pattern = str(batch.color or "")
                    contributions["number_of_opposite_crates"] = (
                        self.coeffs["number_of_opposite_crates"]
                        * (len(pattern) - pattern.count(self.map.robot_color[0]))
                    )
            else:
                contributions["number_of_opposite_crates"] = self.coeffs["number_of_opposite_crates"] * 4.0  # Assume all 4 crates are opposite color for storage pickups

        score = sum(contributions.values())
        return score, contributions

    def _log_score_breakdown(self, task: Task, contributions: Dict[str, float]) -> None:
        """Log score decomposition per task for runtime observability."""
        if not self.log_score_breakdown:
            return

        # message = f"🧮 Score {task.type}:{task.target_id} total={task.score:.3f}"

        # if self.ros_node is not None:
        #     self.ros_node.get_logger().info(message)
        # else:
        #     print(f"[TaskScheduler] {message}")

    def _log_task_queue_snapshot(self, queue: List[Task]) -> None:
        """Log final ordered queue so all candidate tasks are visible."""
        if not self.log_task_queue_snapshot:
            return

        if queue:
            rows = [
                f"#{idx}:{task.type}:{task.target_id}(score={task.score:.3f},options={task.options})"
                for idx, task in enumerate(queue)
            ]
            message = f"🗂️ Task queue ({len(queue)}): " + " | ".join(rows)
            queue_signature = tuple(rows)
        else:
            message = "🗂️ Task queue (0): empty"
            queue_signature = ()

        if queue_signature == self._last_task_queue_signature:
            return

        if self.ros_node is not None:
            self.ros_node.get_logger().info(message)
        else:
            print(f"[TaskScheduler] {message}")
        self._last_task_queue_signature = queue_signature
    
    def choose_side(self, task: Task) -> bool:
        if task.type == "pickup":
            # Safety check: batch may have disappeared due to concurrent updates
            if task.target_id not in self.map.batches:
                return False
            
            pickup_pos = []
            if task.target_id in ["s1", "f1", "s2"]:
                pickup_pos.append(Pose2D(x=self.map.batches[task.target_id].x, y=self.map.batches[task.target_id].y + 175.0))
            elif task.target_id in ["f2", "s4", "f6", "s6", "f9"]:
                pickup_pos.append(Pose2D(x=self.map.batches[task.target_id].x - 175.0, y=self.map.batches[task.target_id].y))
            elif task.target_id in ["s7", "f10", "s8"]:
                pickup_pos.append(Pose2D(x=self.map.batches[task.target_id].x, y=self.map.batches[task.target_id].y - 175.0))
            elif task.target_id in ["f4", "f7"]:
                pickup_pos.append(Pose2D(x=self.map.batches[task.target_id].x + 175.0, y=self.map.batches[task.target_id].y))
            elif task.target_id in ["s3", "s5"]:
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
            best_fallback_pos: Optional[Pose2D] = None
            best_fallback_clearance = -1.0

            sx_positions = [
                (float(b.x), float(b.y), str(bid))
                for bid, b in self.map.batches.items()
                if str(bid).startswith("s")
            ]
            fx_positions = [
                (float(b.x), float(b.y), str(bid))
                for bid, b in self.map.batches.items()
                if str(bid).startswith("f") and str(bid) != str(task.target_id)
            ]

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

                # Keep dropoff point away from SX batches so the robot footprint
                # does not start too close to storage crates after put_down.
                min_clearance_to_sx = float("inf")
                nearest_sx_id = None
                for sx_x, sx_y, sx_id in sx_positions:
                    d = math.hypot(float(candidate.x) - sx_x, float(candidate.y) - sx_y)
                    if d < min_clearance_to_sx:
                        min_clearance_to_sx = d
                        nearest_sx_id = sx_id

                min_clearance_to_fx = float("inf")
                nearest_fx_id = None
                for fx_x, fx_y, fx_id in fx_positions:
                    d = math.hypot(float(candidate.x) - fx_x, float(candidate.y) - fx_y)
                    if d < min_clearance_to_fx:
                        min_clearance_to_fx = d
                        nearest_fx_id = fx_id

                if sx_positions and min_clearance_to_sx < float(self.dropoff_min_clearance_to_sx):
                    # Save best fallback in case no candidate can satisfy the clearance.
                    candidate_clearance = min(min_clearance_to_sx, min_clearance_to_fx)
                    if candidate_clearance > best_fallback_clearance:
                        best_fallback_clearance = candidate_clearance
                        best_fallback_pos = candidate
                    if self.ros_node is not None:
                        self.ros_node.get_logger().debug(
                            f"⏭️ Skip dropoff point ({candidate.x:.1f},{candidate.y:.1f}) for {task.target_id}: "
                            f"too close to {nearest_sx_id} ({min_clearance_to_sx:.1f}mm < {self.dropoff_min_clearance_to_sx:.1f}mm)"
                        )
                    continue

                if fx_positions and min_clearance_to_fx < float(self.dropoff_min_clearance_to_fx):
                    # Save best fallback in case no candidate can satisfy the clearance.
                    candidate_clearance = min(min_clearance_to_sx, min_clearance_to_fx)
                    if candidate_clearance > best_fallback_clearance:
                        best_fallback_clearance = candidate_clearance
                        best_fallback_pos = candidate
                    if self.ros_node is not None:
                        self.ros_node.get_logger().debug(
                            f"⏭️ Skip dropoff point ({candidate.x:.1f},{candidate.y:.1f}) for {task.target_id}: "
                            f"too close to {nearest_fx_id} ({min_clearance_to_fx:.1f}mm < {self.dropoff_min_clearance_to_fx:.1f}mm)"
                        )
                    continue

                # Also validate a point projected toward the drop-off center.
                # This helps reject candidates where near-center motion is blocked.
                to_center_dx = center_x - float(candidate.x)
                to_center_dy = center_y - float(candidate.y)
                to_center_norm = math.hypot(to_center_dx, to_center_dy)
                if to_center_norm > 1e-6:
                    probe_distance = max(0.0, float(self.retreat_distance))
                    toward_center_probe = Pose2D(
                        x=float(candidate.x) + probe_distance * (to_center_dx / to_center_norm),
                        y=float(candidate.y) + probe_distance * (to_center_dy / to_center_norm),
                    )
                    if self.map.check_collision(toward_center_probe, ignore_batch_id=task.target_id):
                        continue
                
                dist = self.map.distance_from(self.map.robot_position, candidate)
                if dist < best_dist:
                    best_dist = dist
                    best_pos = candidate

            if best_pos is None:
                if best_fallback_pos is None:
                    return False
                best_pos = best_fallback_pos
                if self.ros_node is not None:
                    self.ros_node.get_logger().warn(
                        f"⚠️ No dropoff point met SX clearance for {task.target_id}; "
                        f"using best fallback with clearance {best_fallback_clearance:.1f}mm"
                    )

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

        # Log available batches only when snapshot changes.
        if self.map.batches:
            batch_dump = [
                f"{bid}(color={b.color},phi={b.phi_degrees:.1f},x={b.x:.3f},y={b.y:.3f})"
                for bid, b in sorted(self.map.batches.items(), key=lambda item: item[0])
            ]
            message = f"📦 Available batches: {', '.join(batch_dump)}"
            available_signature = tuple(batch_dump)
        else:
            message = "📦 Available batches: none"
            available_signature = ()

        if available_signature != self._last_available_batches_signature:
            if self.ros_node is not None:
                self.ros_node.get_logger().info(message)
            else:
                print(f"[TaskScheduler] {message}")
            self._last_available_batches_signature = available_signature

        queue = []
        queued_pickup_debug_rows = []
        # generate pickup tasks if we have space in the robot
        if self.map.robot_front_empty or self.map.robot_back_empty:
            for batch_id, batch in self.map.batches.items():
                if (self.allow_fx == False) and (batch_id[0] == "f"):
                    if self.ros_node is not None:
                        self.ros_node.get_logger().debug(f"⏭️ Skip pickup {batch_id}: fx pickups disabled")
                    continue
                if batch_id in ["f3", "f5", "f8"]:
                    if self.ros_node is not None:
                        self.ros_node.get_logger().debug(f"⏭️ Skip pickup {batch_id}: reserved center drop-off zone")
                    continue
                if (len(batch.color) != 4) or (batch.color.count(self.map.robot_color[0]) == 4):
                    if self.ros_node is not None:
                        self.ros_node.get_logger().debug(
                            f"⏭️ Skip pickup {batch_id}: invalid/own color pattern='{batch.color}' robot_color='{self.map.robot_color}'"
                        )
                    continue
                if batch_id[0] == "f":
                    possible_pickup_phi = 0.0
                    if batch_id in ["f1", "f10"]:
                        possible_pickup_phi = 90.0
                    if batch.phi_degrees is None or abs(batch.phi_degrees - possible_pickup_phi) > 1.0:
                        if self.ros_node is not None:
                            self.ros_node.get_logger().debug(
                                f"⏭️ Skip pickup {batch_id}: phi mismatch current={batch.phi_degrees} expected={possible_pickup_phi}"
                            )
                        continue
                task = Task(target_id=batch_id, type="pickup")
                task.score, contributions = self._calculate_score_with_breakdown(task)
                self._log_score_breakdown(task, contributions)
                if (self.choose_side(task)):
                    queue.append(task)
                    queued_pickup_debug_rows.append(
                        f"{task.target_id}(score={task.score:.3f},options={task.options})"
                    )
                else:
                    if self.ros_node is not None:
                        self.ros_node.get_logger().debug(
                            f"⏭️ Skip pickup {batch_id}: choose_side returned False"
                        )

        # # Log queued pickup snapshot only when it changes.
        # pickup_signature = tuple(queued_pickup_debug_rows)
        # if pickup_signature != self._last_pickup_queue_signature:
        #     if queued_pickup_debug_rows:
        #         pickup_message = f"🧺 Queued pickup tasks: {', '.join(queued_pickup_debug_rows)}"
        #     else:
        #         pickup_message = "🧺 Queued pickup tasks: none"

        #     if self.ros_node is not None:
        #         self.ros_node.get_logger().info(pickup_message)
        #     else:
        #         print(f"[TaskScheduler] {pickup_message}")
        #     self._last_pickup_queue_signature = pickup_signature
        
        # generate dropoff tasks if we have crates in the robot
        if not self.map.robot_front_empty or not self.map.robot_back_empty:
            for dropoff_id, dropoff in self.map.drop_offs.items():
                if ( not (dropoff_id in ["f3", "f5", "f8"]) or dropoff["status"] != "empty" ):
                    continue
                task = Task(target_id=dropoff_id, type="dropoff")
                task.score, contributions = self._calculate_score_with_breakdown(task)
                self._log_score_breakdown(task, contributions)
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
        go_home_time=max(self.must_go_home_time * self.map.distance_from(self.map.robot_position, self.map.nest_position)/1000.0, 5)

        if self.map.get_time_left() < max(go_home_time, self.must_dropoff_time):
            # If time is running out, prioritize dropoff tasks
            queue = [t for t in queue if t.type == "dropoff"] + [t for t in queue if t.type != "dropoff"]
        
        if (self.map.get_time_left() < max(go_home_time, self.must_dropoff_time)) and len([t for t in queue if t.type == "dropoff"]) == 0:
            task = Task(target_id="nest", type="return_home")
            task.options["drop"] = "none"
            if not self.map.robot_front_empty:
                task.options["drop"] = "front"
            elif not self.map.robot_back_empty:
                task.options["drop"] = "back"
            queue.insert(0, task)  # Prioritize return home task

        self._log_task_queue_snapshot(queue)
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

        # Pre-approach point (same logic as DecisionMaker):
        # apply ±10cm offset depending on batch orientation and whether pickup pose is
        # above/below (horizontal) or left/right (vertical) of batch center.
        offset = max(0.0, float(approach_offset_mm))
        theta_rad = math.radians(float(pickup_theta))
        approach_x = float(pickup_x)
        approach_y = float(pickup_y)

        batch = self.map.batches.get(str(batch_known_id).lower())
        if batch is not None:
            center_x = float(batch.x)
            center_y = float(batch.y)

            # Infer orientation from batch angle (0/180 => horizontal, 90/-90 => vertical)
            phi = float(batch.phi_degrees if batch.phi_degrees is not None else 0.0)
            phi_mod = abs(phi) % 180.0
            if phi_mod <= 45.0 or phi_mod >= 135.0:
                batch_direction = 'horizontal'
            else:
                batch_direction = 'vertical'

            if batch_direction == 'horizontal' and float(pickup_y) < center_y:
                approach_y = float(pickup_y) - offset
            elif batch_direction == 'horizontal' and float(pickup_y) > center_y:
                approach_y = float(pickup_y) + offset
            elif batch_direction == 'vertical' and float(pickup_x) < center_x:
                approach_x = float(pickup_x) - offset
            elif batch_direction == 'vertical' and float(pickup_x) > center_x:
                approach_x = float(pickup_x) + offset

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
        insert_distance_mm: Optional[float] = None,
        retreat_distance_mm: Optional[float] = None,
    ) -> Optional[List[Dict[str, Any]]]:
        """
        Build a put_down task.

        Flow:
        1) Pathfind to (x, y)
        2) Orientate toward target batch center from map.drop_offs
          3) Execute ESP put_down
          4) If type B, create a separate follow-up task with 20cm insertion move
              (front=go_to, back=go_to_backwards), preserving orientation
          5) Always append a final retreat task (10cm by default):
              - front put_down -> move back (`go_to_backwards`)
              - back put_down  -> move forward (`go_to`)

        Args:
            batch_known_id: Putdown zone batch id (e.g., 'f1', 'f5')
            x: Target x in mm
            y: Target y in mm
            device: 'front'/'back' or 'ESP_front'/'ESP_back'
            start_pose: Optional {'x','y','theta'} in mm/degrees for explicit path start
            insert_distance_mm: Insertion move distance for type B zones (default 175mm)
            retreat_distance_mm: Final retreat distance after put_down flow (default 250mm)

        Returns:
            Ordered list of task dicts compatible with strategy_manager queue format:
            - Type A: [putdown_task, putdown_retreat_task]
            - Type B: [putdown_task, putdown_insert_task, putdown_retreat_task]
            None on invalid inputs
        """
        if self.pathfinder is None:
            return None

        if insert_distance_mm is None:
            insert_distance_mm = float(self.putdown_insert_distance)
        if retreat_distance_mm is None:
            retreat_distance_mm = float(self.putdown_retreat_distance)

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

        # Angle at target point to face center of putdown zone.
        # For back putdown, robot heading should be opposite so the BACK side
        # points toward the center while dropping.
        theta_to_center_deg = math.degrees(math.atan2(center_y - target_y, center_x - target_x))
        if esp_device == 'ESP_back':
            theta_to_center_deg += 180.0

        # Normalize to [-180, 180]
        while theta_to_center_deg > 180.0:
            theta_to_center_deg -= 360.0
        while theta_to_center_deg <= -180.0:
            theta_to_center_deg += 360.0
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

        # Orientate to center before put_down (no extra final go_to)
        task_sequence.append({
            'device': 'STM',
            'command': 'orientate',
            'params': [theta_to_center_deg, 0.0],
            'wait': False
        })

        task_sequence.append({
            'device': esp_device,
            'command': 'put_down',
            'params': {
                'from_task_scheduler': True,
                'putdown_insert_distance_mm': float(insert_distance_mm) if zone_type == 'B' else 0.0,
            },
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

        # Final retreat task (always appended as the last task).
        # Retreat direction is away from zone center.
        away_dx = target_x - center_x
        away_dy = target_y - center_y
        away_norm = math.hypot(away_dx, away_dy)
        if away_norm <= 1e-6:
            # Fallback when target ~= center: retreat opposite to facing direction.
            away_dx = -math.cos(theta_to_center_rad)
            away_dy = -math.sin(theta_to_center_rad)
            away_norm = 1.0
        away_ux = away_dx / away_norm
        away_uy = away_dy / away_norm

        # If type B inserts toward center first, retreat starts from insert end-point.
        retreat_start_x = target_x
        retreat_start_y = target_y
        if zone_type == 'B':
            retreat_start_x = target_x + max(0.0, float(insert_distance_mm)) * math.cos(theta_to_center_rad)
            retreat_start_y = target_y + max(0.0, float(insert_distance_mm)) * math.sin(theta_to_center_rad)

        retreat_dist = max(0.0, float(retreat_distance_mm))
        retreat_x = retreat_start_x + retreat_dist * away_ux
        retreat_y = retreat_start_y + retreat_dist * away_uy

        # Requested behavior:
        # - front put_down => move back
        # - back put_down  => move forward
        retreat_stm_command = 'go_to_backwards' if esp_device == 'ESP_front' else 'go_to'

        putdown_retreat_task = {
            'name': f'putdown_retreat_{normalized_id}_{esp_device.split("_")[1]}',
            'sequence': [
                {
                    'device': 'STM',
                    'command': retreat_stm_command,
                    'params': [retreat_x, retreat_y, 900.0, 1.0, {'from_putdown_task': True, 'putdown_retreat': True}],
                    'wait': True
                }
            ],
            'global_goal': {
                'x': retreat_x,
                'y': retreat_y,
                'theta': theta_to_center_deg
            }
        }

        # For type B, build insertion as a separate task executed after put_down
        if zone_type != 'B':
            return [putdown_task, putdown_retreat_task]

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

        return [putdown_task, putdown_insert_task, putdown_retreat_task]

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

    def build_return_home(self) -> Optional[List[Dict[str, Any]]]:
        """
        Build a return-to-home task sequence for end-game.
        
        Sequence:
        1. If robot color is yellow, navigate to intermediate waypoint (803, 879)
        2. Wait for end game (RASP_PI wait_end_game with 12 second param)
        3. Perform bezier backwards movement to home position (251.19, 277.60, θ=0.36)
        
        Returns:
            List of executable task dicts, or None if build fails
        """
        tasks: List[Dict[str, Any]] = []
        robot_color = str(self.map.robot_color or "").lower()
        if robot_color not in ("yellow", "blue"):
            if self.ros_node is not None:
                self.ros_node.get_logger().warning(
                    f"⚠️ Return home: unknown robot color '{robot_color}', defaulting to yellow flow"
                )
            robot_color = "yellow"

        # Blue return-home: path to staging point, wait end-game, then bezier to blue home.
        if robot_color == "blue":
            if self.pathfinder is None:
                if self.ros_node is not None:
                    self.ros_node.get_logger().warning(
                        "⚠️ Return home (blue): pathfinder unavailable for staging waypoint"
                    )
                return None

            blue_stage_x = 899.23
            blue_stage_y = 2197.79
            blue_stage_theta_deg = 90.25191526508233

            stm32_commands, path_success = self.pathfinder.path_full(
                target_x=blue_stage_x / 1000.0,
                target_y=blue_stage_y / 1000.0,
                target_theta=math.radians(blue_stage_theta_deg),
            )

            if not path_success:
                if self.ros_node is not None:
                    self.ros_node.get_logger().warning(
                        "⚠️ Return home (blue): failed to build path to staging waypoint"
                    )
                return None

            intermediate_sequence: List[Dict[str, Any]] = []
            last_cmd = None
            for cmd in stm32_commands:
                values = cmd.replace('go_to(', '').replace(')', '').split(',')
                waypoint_x = float(values[0].strip())
                waypoint_y = float(values[1].strip())
                if cmd != last_cmd:
                    intermediate_sequence.append({
                        'device': 'STM',
                        'command': 'go_to',
                        'params': [waypoint_x, waypoint_y, 1100.0, 0.0, {'from_pathfinding': True, 'from_return_home': True, 'blue_staging': True}],
                        'wait': False
                    })
                last_cmd = cmd

            intermediate_sequence.append({
                'device': 'STM',
                'command': 'go_to',
                'params': [blue_stage_x, blue_stage_y, 1100.0, 1.0, {'from_pathfinding': True, 'from_return_home': True, 'blue_staging': True}],
                'wait': True
            })

            tasks.append({
                'name': 'return_home_intermediate_blue',
                'sequence': intermediate_sequence,
                'global_goal': {
                    'x': blue_stage_x,
                    'y': blue_stage_y,
                    'theta': blue_stage_theta_deg,
                }
            })

            tasks.append({
                'name': 'return_home_wait_end_game',
                'sequence': [
                    {
                        'device': 'RASP_PI',
                        'command': 'wait_end_game',
                        'params': [13],
                        'wait': True
                    }
                ],
                'global_goal': {
                    'x': blue_stage_x,
                    'y': blue_stage_y,
                    'theta': blue_stage_theta_deg,
                }
            })

            tasks.append({
                'name': 'return_home_bezier_blue',
                'sequence': [
                    {
                        'device': 'STM',
                        'command': 'bezier',
                        'params': [
                            '897.99',
                            '2390.29',
                            923.4,
                            2770.13,
                            285,
                            2765.04,
                            1000,
                            1000,
                            1100
                        ],
                        'wait': False
                    }
                ],
                'global_goal': {
                    'x': 285,
                    'y': 2765.04,
                    'theta': -179.5431871268251,
                }
            })

            return tasks

        # Task 1: If yellow, navigate to intermediate waypoint via pathfinding
        if robot_color == "yellow":
            if self.pathfinder is None:
                if self.ros_node is not None:
                    self.ros_node.get_logger().warning(
                        "⚠️ Return home (yellow): pathfinder unavailable for intermediate waypoint"
                    )
                return None

            stm32_commands, path_success = self.pathfinder.path_full(
                target_x=803.0 / 1000.0,
                target_y=879.0 / 1000.0,
                target_theta=0.0,
            )

            if not path_success:
                if self.ros_node is not None:
                    self.ros_node.get_logger().warning(
                        "⚠️ Return home (yellow): failed to build path to intermediate waypoint"
                    )
                return None

            intermediate_sequence: List[Dict[str, Any]] = []
            last_cmd = None
            for cmd in stm32_commands:
                values = cmd.replace('go_to(', '').replace(')', '').split(',')
                waypoint_x = float(values[0].strip())
                waypoint_y = float(values[1].strip())
                if cmd != last_cmd:
                    intermediate_sequence.append({
                        'device': 'STM',
                        'command': 'go_to',
                        'params': [waypoint_x, waypoint_y, 1100.0, 0.0, {'from_pathfinding': True, 'from_return_home': True}],
                        'wait': False
                    })
                last_cmd = cmd

            intermediate_sequence.append({
                'device': 'STM',
                'command': 'go_to',
                'params': [803.0, 879.0, 1100.0, 1.0, {'from_pathfinding': True, 'from_return_home': True}],
                'wait': True
            })

            intermediate_task = {
                'name': 'return_home_intermediate_yellow',
                'sequence': intermediate_sequence,
                'global_goal': {
                    'x': 803.0,
                    'y': 879.0,
                    'theta': 0.0
                }
            }
            tasks.append(intermediate_task)
        
        # Task 2: Wait for end game
        wait_task = {
            'name': 'return_home_wait_end_game',
            'sequence': [
                {
                    'device': 'RASP_PI',
                    'command': 'wait_end_game',
                    'params': [13],
                    'wait': True
                }
            ],
            'global_goal': {
                'x': 803.0,
                'y': 879.0,
                'theta': 0.0
            }
        }
        tasks.append(wait_task)
        
        # Task 3: Bezier backwards movement to home
        home_task = {
            'name': 'return_home_bezier',
            'sequence': [
                {
                    'device': 'STM',
                    'command': 'bezier_backwards',
                    'params': [
                        '791.09',
                        '267.57000000000016',
                        '655.44',
                        '280.14',
                        '251.19',
                        '277.60',
                        1000,
                        1000,
                        1100
                    ],
                    'wait': False
                }
            ],
            'global_goal': {
                'x': '251.19',
                'y': '277.60',
                'theta': 0.3599984288435678
            }
        }
        tasks.append(home_task)
        return tasks

    def get_optimal_task(self, start_pose: Optional[Dict[str, float]] = None) -> Optional[List[Dict[str, Any]]]:
        """
        Pop the best scored high-level task, build executable task(s), and enqueue them in `rpi_queue`.

        Returns:
            List of executable task dict(s) that were enqueued, or None if no valid task.
        """
        task = self.pop_task()
        if task is None:
            if self.ros_node is not None:
                tracked = self.tracked_task.target_id if self.tracked_task is not None else 'none'
                self.ros_node.get_logger().warning(
                    f"⚠️ No task selected: queue is empty | batches={len(self.map.batches)} | tracked={tracked} | front_empty={self.map.robot_front_empty} | back_empty={self.map.robot_back_empty}"
                )
            else:
                tracked = self.tracked_task.target_id if self.tracked_task is not None else 'none'
                print(
                    f"[TaskScheduler] No task selected: queue is empty | batches={len(self.map.batches)} | tracked={tracked} | front_empty={self.map.robot_front_empty} | back_empty={self.map.robot_back_empty}"
                )
            return None

        if self.ros_node is not None:
            self.ros_node.get_logger().info(
                f"🎯 Selected batch: id={task.target_id}, type={task.type}, score={task.score:.3f}"
            )
        else:
            print(f"[TaskScheduler] Selected batch: id={task.target_id}, type={task.type}, score={task.score:.3f}")

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

        elif task.type == 'return_home':
            generated_tasks = self.build_return_home()

        else:
            # Only pickup/dropoff/return_home are converted here.
            return None

        if not generated_tasks:
            self.tracked_task_invalid = True
            if self.ros_node is not None:
                self.ros_node.get_logger().warning(
                    f"⚠️ Failed to build executable task for selected batch id={task.target_id}, type={task.type}"
                )
            else:
                print(f"[TaskScheduler] Failed to build executable task for selected batch id={task.target_id}, type={task.type}")
            return None

        self.rpi_queue.extend(generated_tasks)
        return generated_tasks
    
    def loop(self) -> None:
        """Main loop to update tasks based on the current map state."""
        if hasattr(self.map, 'sync_time_left_from_round'):
            self.map.sync_time_left_from_round()
        self.generate_tasks()
        self.check_tracked_task_validity()

    def get_status_snapshot(self, source: str = "unknown", max_queue_items: int = 20) -> Dict[str, Any]:
        """Return a serializable snapshot of scheduler runtime status for dashboards."""
        queue_items: List[Dict[str, Any]] = []
        for idx, task in enumerate(self.task_queue[:max(0, int(max_queue_items))]):
            queue_items.append({
                'index': int(idx),
                'target_id': str(task.target_id),
                'type': str(task.type),
                'score': float(task.score),
                'options': dict(task.options or {}),
            })

        tracked_task = None
        if self.tracked_task is not None:
            tracked_task = {
                'target_id': str(self.tracked_task.target_id),
                'type': str(self.tracked_task.type),
                'score': float(self.tracked_task.score),
                'options': dict(self.tracked_task.options or {}),
                'invalid': bool(self.tracked_task_invalid),
                'end_action': str(getattr(self.tracked_task, 'end_action', '') or ''),
                'batch_known_id': str(getattr(self.tracked_task, 'batch_known_id', '') or ''),
                'task_name': str(getattr(self.tracked_task, 'task_name', '') or ''),
            }

        time_left_s = None
        if hasattr(self.map, 'get_time_left'):
            try:
                time_left_s = float(self.map.get_time_left())
            except Exception:
                time_left_s = None

        return {
            'source': str(source),
            'time_left_s': time_left_s,
            'front_empty': bool(getattr(self.map, 'robot_front_empty', True)),
            'back_empty': bool(getattr(self.map, 'robot_back_empty', True)),
            'tracked_task': tracked_task,
            'queue_size': int(len(self.task_queue)),
            'queue': queue_items,
            'predicted_enemy_goal': str(self.predicted_enemy_goal),
        }
