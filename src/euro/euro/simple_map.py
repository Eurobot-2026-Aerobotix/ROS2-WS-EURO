#!/usr/bin/env python3
"""
Simple map model for batch, robot, and enemy-relative data.

This file is intentionally minimal and does NOT include logic for
steal/scoring/planning/services/visualization.
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Set
from rclpy.node import Node


class MapUpdater:
    """Background updater for map refresh (batch content is topic-driven)."""

    def __init__(
        self,
        map_ref,
        ros_node,
        pathfinder_client,
        map_update_hz: float = 1.0,
        active_batches_hz: float = 2.0,
    ):
        self.map = map_ref
        self.node = ros_node
        self.pathfinder = pathfinder_client
        self.map_update_hz = max(0.2, min(5.0, float(map_update_hz)))
        self.active_batches_hz = max(0.2, min(10.0, float(active_batches_hz)))

        self._stop_event = threading.Event()
        self._update_lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

        self._enabled = True
        self._warned_missing_update_map = False
        self._last_state_log_s = 0.0
        self._last_update_map_call_s = 0.0
        self._last_synced_batch_count: Optional[int] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name='simple-map-updater', daemon=True)
        self._thread.start()
        self.node.get_logger().info(
            f'🛰️ SimpleMap updater started: update_map={self.map_update_hz:.1f}Hz, '
            f'batches source=/active_obstacles topic'
        )

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = bool(enabled)

    def _run(self) -> None:
        # Drive loop from update_map cadence; batches are updated via topic callback.
        period_s = 1.0 / max(self.map_update_hz, 0.2)
        while not self._stop_event.is_set():
            loop_start = time.perf_counter()
            if self._enabled:
                self.update_once()
            elapsed = time.perf_counter() - loop_start
            self._stop_event.wait(timeout=max(0.0, period_s - elapsed))

    def update_once(self) -> None:
        if not self._update_lock.acquire(blocking=False):
            return

        try:
            now_mono = time.perf_counter()
            do_update_map = (now_mono - self._last_update_map_call_s) >= (1.0 / self.map_update_hz)
            update_map_fn = getattr(self.pathfinder, 'update_map', None)
            if do_update_map and callable(update_map_fn):
                map_success, map_message = update_map_fn()
                self._last_update_map_call_s = now_mono
                if not map_success:
                    self.node.get_logger().warning(f'⚠️ SimpleMap background update_map failed: {map_message}')
            elif do_update_map and not self._warned_missing_update_map:
                self.node.get_logger().warning(
                    '⚠️ Pathfinder has no update_map(); SimpleMap updater relies on /active_obstacles topic only'
                )
                self._warned_missing_update_map = True

        except Exception as exc:
            self.node.get_logger().error(f'❌ SimpleMap background updater exception: {exc}')
        finally:
            self._update_lock.release()

@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

@dataclass
class Speed2D:
    x: float = 0.0
    y: float = 0.0

@dataclass
class EnemyRelativeObstacle:
    """Enemy obstacle received from /obstacle_relative (relative to robot)."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    detected: bool = False


@dataclass
class BatchInfo:
    """Minimal batch information."""
    known_id: str
    x: float
    y: float
    phi_degrees: float = 0.0
    color: str = "unknown"

class Map:
    """
    Clear and simple map class that only stores:
    - Batches
    - Robot position
    - Enemy relative obstacle
    """

    robot_radius = 150.0  # Robot width is 300.
    batch_radius = 100.0
    enemy_radius = 170.0

    def __init__(self, round_timer: Optional[Any] = None, game_duration_s: float = 100.0):
        # Predefined batch positions in millimeters.
        self.predefined_batch_positions: Dict[str, Dict[str, float]] = {
            "s1": {"x": 800.0, "y": 175.0, "phi_degrees": 0.0},
            "s2": {"x": 1600.0, "y": 175.0, "phi_degrees": 0.0},
            "s3": {"x": 1200.0, "y": 1150.0, "phi_degrees": 90.0},
            "s4": {"x": 1825.0, "y": 1100.0, "phi_degrees": 90.0},
            "s5": {"x": 1200.0, "y": 1850.0, "phi_degrees": 90.0},
            "s6": {"x": 1825.0, "y": 1900.0, "phi_degrees": 90.0},
            "s7": {"x": 800.0, "y": 2800.0, "phi_degrees": 90.0},
            "s8": {"x": 1600.0, "y": 2800.0, "phi_degrees": 90.0},
        }
        self.pickup_poses = {
            'S1': [{'x': 800, 'y': 400, 'theta': [90, -90]}],
            'S2': [{'x': 1600, 'y': 400, 'theta': [90, -90]}],
            'S3': [
                {'x': 1430, 'y': 1150, 'theta': [0, 180]},
                {'x': 970, 'y': 1150, 'theta': [0, 180]}
            ],
            'S4': [{'x': 1600, 'y': 1100, 'theta': [0, 180]}],
            'S5': [
                {'x': 1430, 'y': 1850, 'theta': [0, 180]},
                {'x': 970, 'y': 1850, 'theta': [0, 180]}
            ],
            'S6': [{'x': 1600, 'y': 1900, 'theta': [0, 180]}],
            'S8': [{'x': 1600, 'y': 2600, 'theta': [0, 180]}],
            'S7': [{'x': 800, 'y': 2600, 'theta': [0, 180]}]
        }

        self.drop_offs: Dict[str, Dict[str, float]] = {
            "f1": {"x": 1200, "y": 100, "type": "left", "batch_type": "A", "status": "empty"},
            "f2": {"x": 1900, "y": 700, "type": "front", "batch_type": "A", "status": "empty"},
            "f3": {"x": 1200, "y": 800, "type": "center", "batch_type": "B", "status": "empty"},
            "f4": {"x": 550, "y": 1250, "type": "back", "batch_type": "A", "status": "empty"},
            "f5": {"x": 1200, "y": 1500, "type": "center", "batch_type": "B", "status": "empty"},
            "f6": {"x": 1900, "y": 1500, "type": "front", "batch_type": "A", "status": "empty"},
            "f7": {"x": 550, "y": 1750, "type": "back", "batch_type": "A", "status": "empty"},
            "f8": {"x": 1200, "y": 2200, "type": "center", "batch_type": "B", "status": "empty"},
            "f9": {"x": 1900, "y": 2300, "type": "front", "batch_type": "A", "status": "empty"},
            "f10": {"x": 1200, "y": 2900, "type": "right", "batch_type": "A", "status": "empty"},
        }

        self.batches: Dict[str, BatchInfo] = {}
        # Persistently ignored batch ids (normalized to lowercase).
        # Any id in this set will be filtered out from get_active_batches sync
        # to avoid re-adding batches that were already picked/deleted locally.
        self.deleted_batch_ids: Set[str] = set()
        self.robot_position: Pose2D = Pose2D()
        self.enemy_relative: EnemyRelativeObstacle = EnemyRelativeObstacle()
        self.enemy_absolute: Pose2D = Pose2D()
        self.enemy_speed: Speed2D = Speed2D()
        self.time_left = 100.0
        self.round_timer = round_timer
        self.game_duration_s = float(game_duration_s)
        self.robot_color = "unknown"
        self.robot_front_empty = True
        self.robot_back_empty = True
        self.nest_position = Pose2D(x=200.0, y=2700.0)
        self.ninja_on_map = True
        self._subscriptions = []
        self._update_map_timer = None
        self._round_update_start_s = 5.0
        self._round_update_end_s = 100.0
        self._pathfinder_client = None
        self.map_updater: Optional[MapUpdater] = None
        self._active_obstacles_sub = None
        self._last_topic_batches_update_s = 0.0
        self._warned_missing_active_obstacles_msg = False

    @staticmethod
    def _normalize_batch_id(known_id: Optional[str]) -> str:
        return str(known_id or '').strip().lower()

    def has_recent_topic_batches_update(self, max_age_s: float = 3.0) -> bool:
        if self._last_topic_batches_update_s <= 0.0:
            return False
        return (time.time() - self._last_topic_batches_update_s) <= float(max_age_s)

    def update_batches_from_active_batches_msg(self, msg: Any) -> int:
        """Update internal `batches` from /active_obstacles (hybrid_astar/msg/ActiveBatches)."""
        batch_info = {
            'centers': list(getattr(msg, 'centers', []) or []),
            'known_ids': list(getattr(msg, 'known_ids', []) or []),
            'batch_types': list(getattr(msg, 'batch_types', []) or []),
            'member_counts': list(getattr(msg, 'member_counts', []) or []),
            'directions': list(getattr(msg, 'directions', []) or []),
            'patterns': list(getattr(msg, 'patterns', []) or []),
            'theta_degrees': list(getattr(msg, 'theta_degrees', []) or []),
            'qualities': list(getattr(msg, 'qualities', []) or []),
        }

        updated = self.update_batches_from_active_batch_info(batch_info)
        self._last_topic_batches_update_s = time.time()
        return updated

    def _active_obstacles_callback(self, msg) -> None:
        updated_count = self.update_batches_from_active_batches_msg(msg)
        now = time.time()
        if self.map_updater is not None:
            if self.map_updater._last_synced_batch_count != updated_count:
                if hasattr(self, 'get_logger'):
                    self.get_logger().info(f'🧠 SimpleMap batch sync: {updated_count} batches')
                self.map_updater._last_synced_batch_count = updated_count
                self.map_updater._last_state_log_s = now
            elif now - self.map_updater._last_state_log_s >= 10.0:
                if hasattr(self, 'get_logger'):
                    self.get_logger().debug(f'🧠 SimpleMap batch sync unchanged: {updated_count} batches')
                self.map_updater._last_state_log_s = now

    def update_batches_from_active_batch_info(self, batch_info: Dict[str, List[Any]]) -> int:
        """Update internal `batches` map from pathfinder `get_active_batches()` response."""
        required = ['known_ids', 'member_counts', 'directions', 'centers']
        for key in required:
            if key not in batch_info:
                return 0

        known_ids = batch_info.get('known_ids', [])
        member_counts = batch_info.get('member_counts', [])
        directions = batch_info.get('directions', [])
        patterns = batch_info.get('patterns', [])
        theta_degrees_list = batch_info.get('theta_degrees', [])
        centers = batch_info.get('centers', [])

        n = min(len(known_ids), len(member_counts), len(directions), len(centers))

        # Rebuild active list: only keep batches that still have members.
        next_batches: Dict[str, BatchInfo] = {}
        for i in range(n):
            member_count = int(member_counts[i])
            if member_count <= 0:
                continue

            known_id = self._normalize_batch_id(known_ids[i])
            if not known_id or known_id in self.deleted_batch_ids:
                continue
            direction = str(directions[i]).lower()
            center = centers[i]
            if i < len(theta_degrees_list):
                phi_degrees = float(theta_degrees_list[i])
            else:
                phi_degrees = 0.0 if direction == 'horizontal' else (90.0 if direction == 'vertical' else 0.0)

            pattern = str(patterns[i]).lower() if i < len(patterns) else ''
            batch_color = pattern if pattern else 'unknown'

            next_batches[known_id] = BatchInfo(
                known_id=known_id,
                x=float(center.x) * 1000.0,
                y=float(center.y) * 1000.0,
                phi_degrees=phi_degrees,
                color=batch_color,
            )

        self.batches = next_batches
        return len(self.batches)

    def set_round_timer(self, round_timer: Any, game_duration_s: Optional[float] = None) -> None:
        """Attach a round timer object that exposes `time()` in seconds."""
        self.round_timer = round_timer
        if game_duration_s is not None:
            self.game_duration_s = float(game_duration_s)

    def sync_time_left_from_round(self) -> None:
        """Update `time_left` from attached round timer, if available."""
        if self.round_timer is None or not hasattr(self.round_timer, 'time'):
            return
        elapsed = float(self.round_timer.time())
        self.time_left = max(0.0, self.game_duration_s - elapsed)

    def _update_map_during_round(self) -> None:
        """Timer callback: sync time-left and gate background updater during configured round window."""
        self.sync_time_left_from_round()

        if self.map_updater is None:
            return

        if self.round_timer is None:
            self.map_updater.set_enabled(True)
            return

        if hasattr(self.round_timer, 'has_started') and not self.round_timer.has_started():
            self.map_updater.set_enabled(True)
            return

        if not hasattr(self.round_timer, 'time'):
            self.map_updater.set_enabled(True)
            return

        elapsed = float(self.round_timer.time())
        self.map_updater.set_enabled(self._round_update_start_s <= elapsed <= self._round_update_end_s)

    def attach_round_update_timer(
        self,
        node,
        period_s: float = 5.0,
        start_s: float = 5.0,
        end_s: float = 100.0,
        pathfinder_client: Optional[Any] = None,
    ) -> None:
        """
        Attach periodic timer on ROS node to refresh map timing during round.

        Default active window: [5s, 100s].
        """
        self._round_update_start_s = float(start_s)
        self._round_update_end_s = float(end_s)
        self._pathfinder_client = pathfinder_client

        if self.map_updater is None and pathfinder_client is not None:
            self.map_updater = MapUpdater(
                map_ref=self,
                ros_node=node,
                pathfinder_client=pathfinder_client,
                map_update_hz=0.05,
                active_batches_hz=0.2,
            )
            self.map_updater.set_enabled(True)
            self.map_updater.start()

        if self._update_map_timer is not None:
            return

        self._update_map_timer = node.create_timer(float(period_s), self._update_map_during_round)

    def stop_map_updater(self) -> None:
        if self.map_updater is not None:
            self.map_updater.stop()

    # -----------------------------
    # Base setters / state updates
    # -----------------------------

    def set_robot_position(self, x: float, y: float, theta: float = 0.0) -> None:
        self.robot_position = Pose2D(x=x, y=y, theta=theta)

    def update_robot_front_status(self, empty: bool) -> None:
        self.robot_front_empty = empty

    def update_robot_back_status(self, empty: bool) -> None:
        self.robot_back_empty = empty

    def set_enemy_relative(self, x: float, y: float, theta: float = 0.0, detected: bool = True) -> None:
        self.enemy_relative = EnemyRelativeObstacle(x=x, y=y, theta=theta, detected=detected)
        self.update_enemy_absolute()

    def clear_enemy_relative(self) -> None:
        self.enemy_relative = EnemyRelativeObstacle()
        self.enemy_absolute = Pose2D()  # Clear absolute when relative is cleared
    
    def set_enemy_speed(self, x: float, y: float) -> None: #TODO: check if enemy speed is relative or absolute
        self.enemy_speed = Speed2D(x=x, y=y)
    
    def update_enemy_absolute(self) -> None:
        """Update enemy absolute position based on robot position and enemy relative position."""
        if not self.enemy_relative.detected:
            self.enemy_absolute = Pose2D()  # Clear absolute if enemy not detected
            return
        
        # Simple transformation from relative to absolute (assuming no rotation for simplicity)
        # Transform enemy relative position to absolute using robot's orientation
        cos_theta = math.cos(math.radians(self.robot_position.theta))
        sin_theta = math.sin(math.radians(self.robot_position.theta))
        
        # Rotate enemy relative position by robot's theta
        rotated_x = self.enemy_relative.x * cos_theta - self.enemy_relative.y * sin_theta
        rotated_y = self.enemy_relative.x * sin_theta + self.enemy_relative.y * cos_theta
        
        self.enemy_absolute = Pose2D(
            x=self.robot_position.x + rotated_x,
            y=self.robot_position.y + rotated_y,
            theta=None,
        )

    def set_batch(self, known_id: str, x: float, y: float, phi_degrees: float = 0.0, color: str = "unknown") -> None:
        normalized_id = self._normalize_batch_id(known_id)
        if not normalized_id:
            return
        if normalized_id in self.deleted_batch_ids:
            return

        self.batches[normalized_id] = BatchInfo(
            known_id=normalized_id,
            x=x,
            y=y,
            phi_degrees=phi_degrees,
            color=color,
        )
        if normalized_id[0] == "f" and normalized_id in self.drop_offs:
            if color != "unknown":
                self.drop_offs[normalized_id]["status"] = "full"

    def set_batch_from_predefined(self, known_id: str, color: str = "unknown") -> bool:
        """Create/update batch from predefined positions. Returns False if id is unknown."""
        normalized_id = self._normalize_batch_id(known_id)
        if not normalized_id or normalized_id in self.deleted_batch_ids:
            return False
        if normalized_id not in self.predefined_batch_positions:
            return False

        p = self.predefined_batch_positions[normalized_id]
        self.set_batch(
            known_id=normalized_id,
            x=p["x"],
            y=p["y"],
            phi_degrees=p["phi_degrees"],
            color=color,
        )
        return True

    def remove_batch(self, known_id: str, persistently: bool = True) -> None:
        normalized_id = self._normalize_batch_id(known_id)
        if not normalized_id:
            return

        self.batches.pop(normalized_id, None)
        if persistently:
            self.deleted_batch_ids.add(normalized_id)

        if normalized_id[0] == "f" and normalized_id in self.drop_offs:
            self.drop_offs[normalized_id]["status"] = "empty"

    def clear_deleted_batch_ids(self) -> None:
        self.deleted_batch_ids.clear()

    def clear_batches(self) -> None:
        self.batches.clear()
        for dropoff in self.drop_offs.values():
            dropoff["status"] = "empty"

    def get_batch(self, known_id: str) -> Optional[BatchInfo]:
        normalized_id = self._normalize_batch_id(known_id)
        return self.batches.get(normalized_id)
    
    def update_time_left(self, time_left: float) -> None:
        self.time_left = time_left
    
    def get_time_left(self) -> float:
        return self.time_left
    
    def set_robot_color(self, color: str) -> None:
        self.robot_color = color
        if color == "blue":
            self.nest_position = Pose2D(x=200.0, y=2700.0)
    
    def set_ninja_status(self, on_map: bool) -> None:
        self.ninja_on_map = on_map

    def distance_from(self, point_1: Pose2D, point_2: Pose2D) -> float:
        return ((point_1.x - point_2.x) ** 2 + (point_1.y - point_2.y) ** 2) ** 0.5

    def check_collision(self, robot_pos: Pose2D, ignore_batch_id: Optional[str] = None, safety_margin: float = 0.0) -> bool:
        """Return True when a robot placed at `robot_pos` intersects enemy or batch obstacles."""
        px = float(robot_pos.x)
        py = float(robot_pos.y)
        clearance = self.robot_radius + float(safety_margin)

        ignored_id = str(ignore_batch_id).lower() if ignore_batch_id is not None else None
        for batch in self.batches.values():
            if ignored_id is not None and str(batch.known_id).lower() == ignored_id:
                continue
            if math.hypot(px - float(batch.x), py - float(batch.y)) <= (clearance + self.batch_radius):
                return True

        if self.enemy_relative.detected:
            if math.hypot(px - float(self.enemy_absolute.x), py - float(self.enemy_absolute.y)) <= (clearance + self.enemy_radius):
                return True

        return False

    def calculate_number_of_obstacles_to(self, target_pos: Pose2D) -> int:
        """Calculate the number of obstacles between the robot and a target position."""

        start_x = float(self.robot_position.x)
        start_y = float(self.robot_position.y)
        end_x = float(target_pos.x)
        end_y = float(target_pos.y)

        dx = end_x - start_x
        dy = end_y - start_y
        seg_len_sq = dx * dx + dy * dy

        def _distance_point_to_segment(px: float, py: float) -> float:
            if seg_len_sq <= 1e-9:
                return math.hypot(px - start_x, py - start_y)
            t = ((px - start_x) * dx + (py - start_y) * dy) / seg_len_sq
            t = max(0.0, min(1.0, t))
            closest_x = start_x + t * dx
            closest_y = start_y + t * dy
            return math.hypot(px - closest_x, py - closest_y)

        obstacle_count = 0

        for batch in self.batches.values():
            # Skip the destination batch itself when targeting a pickup point.
            if math.hypot(batch.x - end_x, batch.y - end_y) <= 1e-6:
                continue
            if _distance_point_to_segment(float(batch.x), float(batch.y)) <= (self.robot_radius + self.batch_radius):
                obstacle_count += 1

        if self.enemy_relative.detected:
            if _distance_point_to_segment(float(self.enemy_absolute.x), float(self.enemy_absolute.y)) <= (self.robot_radius + self.enemy_radius):
                obstacle_count += 1

        return obstacle_count

    # -----------------------------
    # ROS topic integration
    # -----------------------------

    def _robot_pose_callback(self, msg) -> None:
        """/robot_pose_raw callback (geometry_msgs/msg/Vector3)."""
        self.set_robot_position(float(msg.x) * 1000.0, float(msg.y) * 1000.0, float(msg.z))

    def _enemy_relative_callback(self, msg) -> None:
        """/obstacle_relative callback (geometry_msgs/msg/PoseStamped)."""
        
        self.set_enemy_relative(
            float(msg.pose.position.x) * 1000.0,
            float(msg.pose.position.y) * 1000.0,
            theta=0.0,
            detected=True,
        )

    def _enemy_speed_callback(self, msg) -> None:
        """/obstacle_relative_speed callback (geometry_msgs/msg/Twist)."""
        self.set_enemy_speed(float(msg.linear.x), float(msg.linear.y))

    def attach_ros_subscribers(self, node, qos: int = 10) -> None:
        """
        Attach map setters to ROS topics.

        Topics:
        - /robot_pose_raw           -> set_robot_position (Vector3)
        - /obstacle_relative        -> set_enemy_relative (PoseStamped)
        - /obstacle_relative_speed  -> set_enemy_speed (Twist)
        """
        from geometry_msgs.msg import PoseStamped, Twist, Vector3

        self._subscriptions.append(
            node.create_subscription(Vector3, '/robot_pose_raw', self._robot_pose_callback, qos)
        )
        self._subscriptions.append(
            node.create_subscription(PoseStamped, '/obstacle_relative', self._enemy_relative_callback, qos)
        )
        self._subscriptions.append(
            node.create_subscription(Twist, '/obstacle_relative_speed', self._enemy_speed_callback, qos)
        )

        # Topic-driven active batch map updates from hybrid_astar
        # (replaces frequent get_active_batches polling when available).
        try:
            from hybrid_astar.msg import ActiveBatches  # type: ignore
            self._active_obstacles_sub = node.create_subscription(
                ActiveBatches,
                '/active_obstacles',
                self._active_obstacles_callback,
                qos,
            )
            self._subscriptions.append(self._active_obstacles_sub)
        except Exception as exc:
            if not self._warned_missing_active_obstacles_msg:
                node.get_logger().warning(
                    f'⚠️ Could not subscribe to /active_obstacles topic (hybrid_astar.msg unavailable): {exc}'
                )
                self._warned_missing_active_obstacles_msg = True


class MapNode(Map, Node):
    """
    ROS2 node version of `Map`.

    This keeps all map data/methods directly on the node instance and
    subscribes to map-update topics during initialization.
    """

    def __init__(
        self,
        node_name: str = 'simple_map_node',
        qos: int = 10,
        round_timer: Optional[Any] = None,
        game_duration_s: float = 100.0,
    ):
        Node.__init__(self, node_name)
        Map.__init__(self, round_timer=round_timer, game_duration_s=game_duration_s)
        self.attach_ros_subscribers(self, qos=qos)