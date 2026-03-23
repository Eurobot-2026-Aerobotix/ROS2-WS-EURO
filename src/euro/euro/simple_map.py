#!/usr/bin/env python3
"""
Simple map model for batch, robot, and enemy-relative data.

This file is intentionally minimal and does NOT include logic for
steal/scoring/planning/services/visualization.
"""

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional


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

    def __init__(self):
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
            "f1": {"x": 1.2, "y": 0.1, "type": "left", "status": "empty"},
            "f2": {"x": 1.9, "y": 0.7, "type": "front", "status": "empty"},
            "f3": {"x": 1.2, "y": 0.8, "type": "center", "status": "empty"},
            "f4": {"x": 0.55, "y": 1.25, "type": "back", "status": "empty"},
            "f5": {"x": 1.2, "y": 1.5, "type": "center", "status": "empty"},
            "f6": {"x": 1.9, "y": 1.5, "type": "front", "status": "empty"},
            "f7": {"x": 0.55, "y": 1.75, "type": "back", "status": "empty"},
            "f3": {"x": 1.2, "y": 2.2, "type": "center", "status": "empty"},
            "f2": {"x": 1.9, "y": 2.3, "type": "front", "status": "empty"},
            "f1": {"x": 1.2, "y": 2.9, "type": "right", "status": "empty"},
        }

        self.batches: Dict[str, BatchInfo] = {}
        self.robot_position: Pose2D = Pose2D()
        self.enemy_relative: EnemyRelativeObstacle = EnemyRelativeObstacle()
        self.enemy_absolute: Pose2D = Pose2D()
        self.enemy_speed: Speed2D = Speed2D()
        self.time_left = 100.0
        self.robot_color = "unknown"

    # -----------------------------
    # Base setters / state updates
    # -----------------------------

    def set_robot_position(self, x: float, y: float, theta: float = 0.0) -> None:
        self.robot_position = Pose2D(x=x, y=y, theta=theta)

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