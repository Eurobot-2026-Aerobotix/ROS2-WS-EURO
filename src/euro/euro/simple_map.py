#!/usr/bin/env python3
"""
Simple map model for batch, robot, and enemy-relative data.

This file is intentionally minimal and does NOT include logic for
steal/scoring/planning/services/visualization.
"""

import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional
from rclpy.node import Node

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

    def __init__(self, round_timer: Optional[Any] = None, game_duration_s: float = 100.0):
        # Predefined batch positions from current hybrid_astar map
        self.predefined_batch_positions: Dict[str, Dict[str, float]] = {
            "s1": {"x": 0.80, "y": 0.175, "phi_degrees": 0.0},
            "s2": {"x": 1.60, "y": 0.175, "phi_degrees": 0.0},
            "s3": {"x": 1.20, "y": 1.15, "phi_degrees": 90.0},
            "s4": {"x": 1.825, "y": 1.1, "phi_degrees": 90.0},
            "s5": {"x": 1.20, "y": 1.85, "phi_degrees": 90.0},
            "s6": {"x": 1.825, "y": 1.9, "phi_degrees": 90.0},
            "s7": {"x": 0.80, "y": 2.8, "phi_degrees": 90.0},
            "s8": {"x": 1.60, "y": 2.8, "phi_degrees": 90.0},
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
            'S7': [{'x': 1600, 'y': 2600, 'theta': [0, 180]}],
            'S8': [{'x': 800, 'y': 2600, 'theta': [0, 180]}]
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
        self.nest_position = Pose2D(x=0.2, y=2.7)  # Assuming nest is at origin for simplicity
        self._subscriptions = []
        self._update_map_timer = None
        self._round_update_start_s = 5.0
        self._round_update_end_s = 100.0
        self._pathfinder_client = None

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
        """Timer callback: update map timing only during configured round window."""
        if self.round_timer is None:
            return
        if hasattr(self.round_timer, 'has_started') and not self.round_timer.has_started():
            return
        if not hasattr(self.round_timer, 'time'):
            return

        elapsed = float(self.round_timer.time())
        if self._round_update_start_s <= elapsed <= self._round_update_end_s:
            if self._pathfinder_client is not None and hasattr(self._pathfinder_client, 'update_map'):
                try:
                    self._pathfinder_client.update_map()
                except Exception:
                    # Keep timer robust even if service call fails.
                    pass

    def attach_round_update_timer(
        self,
        node,
        period_s: float = 1.0,
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

        if self._update_map_timer is not None:
            return

        self._update_map_timer = node.create_timer(float(period_s), self._update_map_during_round)

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
        self.batches[known_id] = BatchInfo(
            known_id=known_id,
            x=x,
            y=y,
            phi_degrees=phi_degrees,
            color=color,
        )

    def set_batch_from_predefined(self, known_id: str, color: str = "unknown") -> bool:
        """Create/update batch from predefined positions. Returns False if id is unknown."""
        if known_id not in self.predefined_batch_positions:
            return False

        p = self.predefined_batch_positions[known_id]
        self.set_batch(
            known_id=known_id,
            x=p["x"],
            y=p["y"],
            phi_degrees=p["phi_degrees"],
            color=color,
        )
        return True

    def remove_batch(self, known_id: str) -> None:
        self.batches.pop(known_id, None)

    def clear_batches(self) -> None:
        self.batches.clear()

    def get_batch(self, known_id: str) -> Optional[BatchInfo]:
        return self.batches.get(known_id)

    def update_drop_off_status(self, drop_off_id: str, status: str) -> None:
        if drop_off_id in self.drop_offs:
            self.drop_offs[drop_off_id]["status"] = status
    
    def update_time_left(self, time_left: float) -> None:
        self.time_left = time_left
    
    def get_time_left(self) -> float:
        return self.time_left
    
    def set_robot_color(self, color: str) -> None:
        self.robot_color = color
        if color == "blue":
            self.nest_position = Pose2D(x=0.2, y=2.7)  # Blue nest at origin

    def distance_from(self, point_1: Pose2D, point_2: Pose2D) -> float:
        return ((point_1.x - point_2.x) ** 2 + (point_1.y - point_2.y) ** 2) ** 0.5

    # -----------------------------
    # ROS topic integration
    # -----------------------------

    def _robot_pose_callback(self, msg) -> None:
        """/robot_pose_raw callback (geometry_msgs/msg/Vector3)."""
        self.set_robot_position(float(msg.x), float(msg.y), float(msg.z))

    def _enemy_relative_callback(self, msg) -> None:
        """/obstacle_relative callback (geometry_msgs/msg/PoseStamped)."""
        
        self.set_enemy_relative(
            float(msg.pose.position.x),
            float(msg.pose.position.y),
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