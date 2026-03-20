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

        self.batches: Dict[str, BatchInfo] = {}
        self.robot_position: Pose2D = Pose2D()
        self.enemy_relative: EnemyRelativeObstacle = EnemyRelativeObstacle()

    # -----------------------------
    # Base setters / state updates
    # -----------------------------

    def set_robot_position(self, x: float, y: float, theta: float = 0.0) -> None:
        self.robot_position = Pose2D(x=x, y=y, theta=theta)

    def set_enemy_relative(self, x: float, y: float, theta: float = 0.0, detected: bool = True) -> None:
        self.enemy_relative = EnemyRelativeObstacle(x=x, y=y, theta=theta, detected=detected)

    def clear_enemy_relative(self) -> None:
        self.enemy_relative = EnemyRelativeObstacle()

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