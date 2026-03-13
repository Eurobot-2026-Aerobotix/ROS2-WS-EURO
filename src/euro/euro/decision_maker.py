#!/usr/bin/env python3
"""
Decision Maker Module for Dynamic Batch Stealing

This module generates dynamic tasks to steal batches when:
- Path generation is impossible
- Someone steals a batch we're going to pick up
- Static batches are finished

Features:
- Tracks front/back picker status (full/empty)
- Maintains predefined pickup poses for SX batches (S1-S8)
- Queries available batches from map.py via pathfinder
- Generates complete tasks with STM32 waypoints and ESP32 commands
"""

import math
import threading
from typing import Dict, List, Optional, Tuple
from geometry_msgs.msg import Point
import time


def circular_angle_difference(angle1: float, angle2: float) -> float:
    """
    Calculate the minimum angular difference between two angles in degrees.
    Handles wrap-around at ±180°.
    
    Args:
        angle1: First angle in degrees
        angle2: Second angle in degrees
    
    Returns:
        Minimum absolute difference in degrees (0-180)
    """
    diff = abs(angle1 - angle2)
    if diff > 180:
        diff = 360 - diff
    return diff


# Predefined pickup poses for SX batches (positions in mm)
PICKUP_POSES = {
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

# Predefined putdown poses for FX zones (positions in mm)
PUTDOWN_POSES = {
    'fx1': {'x': 1200, 'y': 100, 'type': 'A'},
    'fx2': {'x': 1900, 'y': 700, 'type': 'A'},
    'fx3': {'x': 1200, 'y': 800, 'type': 'B'},
    'fx5': {'x': 1200, 'y': 1500, 'type': 'B'},
    'fx6': {'x': 1900, 'y': 1500, 'type': 'A'},
    'fx8': {'x': 1200, 'y': 2200, 'type': 'B'},
    'fx9': {'x': 1900, 'y': 2300, 'type': 'A'},
    'fx10': {'x': 1200, 'y': 2900, 'type': 'A'}
}


class DecisionMaker:
    """
    Decision maker that generates dynamic tasks to steal batches
    """
    
    def __init__(self, strategy_manager, pathfinder_client):
        """
        Initialize the decision maker
        
        Args:
            strategy_manager: StrategyManager instance (for logging and state access)
            pathfinder_client: PathfinderClient instance to query batches and generate paths
        """
        self.strategy = strategy_manager
        self.pathfinder = pathfinder_client
        
        # Track picker status (True = full, False = empty)
        self.front_full = False
        self.back_full = False
        
        # Cache for available batches
        self.available_batches = {}
        self.last_batch_update = 0.0
        self.last_update_map_second = -1
        self._update_map_lock = threading.Lock()

        # Enemy proximity thresholds (in mm)
        # - hard_block_radius_mm: batch is rejected if enemy is this close
        # - near_penalty_radius_mm: batch score is penalized if enemy is within this radius
        self.hard_block_radius_mm = 280.0
        self.near_penalty_radius_mm = 520.0

        self._update_map_timer = self.strategy.create_timer(
            1.0,
            self._update_map_during_round
        )
        
        self.strategy.get_logger().info('🧠 Decision Maker initialized')

    def _update_map_during_round(self):
        """Refresh the map once per elapsed game second until the round reaches 100s."""
        round_timer = getattr(self.strategy, 'round', None)
        if round_timer is None or not round_timer.has_started():
            return

        elapsed = round_timer.time()
        if elapsed >= 100.0:
            return

        current_second = int(elapsed)
        if current_second <= self.last_update_map_second:
            return

        if not self._update_map_lock.acquire(blocking=False):
            return

        try:
            if current_second <= self.last_update_map_second:
                return

            self.last_update_map_second = current_second
            success, message = self.pathfinder.update_map()
            if not success:
                self.strategy.get_logger().warning(
                    f'⚠️ Periodic update_map failed at t={current_second}s: {message}'
                )
        finally:
            self._update_map_lock.release()

    def _get_enemy_position_mm(self) -> Optional[Tuple[float, float]]:
        """Get latest enemy position from pathfinder in mm, or None if unavailable."""
        if self.pathfinder.enemy_x is None or self.pathfinder.enemy_y is None:
            return None
        return (self.pathfinder.enemy_x * 1000.0, self.pathfinder.enemy_y * 1000.0)

    @staticmethod
    def _distance_mm(x1: float, y1: float, x2: float, y2: float) -> float:
        """Euclidean distance in mm."""
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def _get_batch_enemy_distances_mm(self, batch: Dict, pickup_pose: Dict,
                                      enemy_x_mm: float, enemy_y_mm: float) -> Tuple[float, float, float]:
        """
        Returns distances from enemy to:
          (batch_center_mm, pickup_pose_mm, min_of_both_mm)
        """
        # batch center is in meters in active batch info
        center_x_mm = batch['center'].x * 1000.0
        center_y_mm = batch['center'].y * 1000.0

        pickup_x_mm = float(pickup_pose['x'])
        pickup_y_mm = float(pickup_pose['y'])

        d_center = self._distance_mm(enemy_x_mm, enemy_y_mm, center_x_mm, center_y_mm)
        d_pickup = self._distance_mm(enemy_x_mm, enemy_y_mm, pickup_x_mm, pickup_y_mm)
        return d_center, d_pickup, min(d_center, d_pickup)

    def _is_hard_blocked_by_enemy(self, batch: Dict, pickup_pose: Dict,
                                  enemy_x_mm: float, enemy_y_mm: float) -> bool:
        """Return True if enemy is too close to batch center or pickup pose."""
        _, _, min_distance = self._get_batch_enemy_distances_mm(
            batch, pickup_pose, enemy_x_mm, enemy_y_mm
        )
        return min_distance <= self.hard_block_radius_mm

    def _compute_enemy_penalty(self, batch: Dict, pickup_pose: Dict,
                               enemy_x_mm: float, enemy_y_mm: float) -> float:
        """
        Compute a score penalty based on enemy proximity.
        0 outside near_penalty_radius_mm, increasing as enemy gets closer.
        """
        _, _, min_distance = self._get_batch_enemy_distances_mm(
            batch, pickup_pose, enemy_x_mm, enemy_y_mm
        )

        if min_distance >= self.near_penalty_radius_mm:
            return 0.0

        # Guard against invalid configuration
        effective_span = max(1.0, self.near_penalty_radius_mm - self.hard_block_radius_mm)

        # proximity in [0,1], where 1 is right at hard block radius
        proximity = (self.near_penalty_radius_mm - max(min_distance, self.hard_block_radius_mm)) / effective_span

        max_penalty = 2.0
        return proximity * max_penalty
    
    def set_front_status(self, is_full: bool):
        """Set front picker status"""
        self.front_full = is_full
        status = "FULL" if is_full else "EMPTY"
        self.strategy.get_logger().info(f"🔧 Front picker: {status}")
    
    def set_back_status(self, is_full: bool):
        """Set back picker status"""
        self.back_full = is_full
        status = "FULL" if is_full else "EMPTY"
        self.strategy.get_logger().info(f"🔧 Back picker: {status}")
    
    def is_front_available(self) -> bool:
        """Check if front picker is available (empty)"""
        return not self.front_full
    
    def is_back_available(self) -> bool:
        """Check if back picker is available (empty)"""
        return not self.back_full
    
    def get_available_device(self) -> Optional[str]:
        """
        Get an available device (front or back) for pickup.
        Simple fallback when no angle optimization is possible.
        
        Returns:
            'ESP_front' or 'ESP_back' if available, None if both full
        """
        if self.is_front_available():
            return 'ESP_front'
        elif self.is_back_available():
            return 'ESP_back'
        else:
            self.strategy.get_logger().warning("⚠️ Both front and back pickers are full!")
            return None
    
    def get_optimal_device_from_path(self, stm32_commands: List[str]) -> Optional[str]:
        """
        Select the optimal device (front or back) based on actual path trajectory.
        Analyzes the initial direction of movement to minimize rotations.
        
        Args:
            stm32_commands: List of path waypoint commands from pathfinder
        
        Returns:
            'ESP_front', 'ESP_back', or None if both full
        """
        # Get current robot pose
        robot_pose = self.strategy.latest_robot_pose
        if robot_pose is None:
            self.strategy.get_logger().warning('⚠️ Robot pose unavailable - using simple device selection')
            return self.get_available_device()
        
        if not stm32_commands:
            self.strategy.get_logger().warning('⚠️ No path waypoints - using simple device selection')
            return self.get_available_device()
        
        # Parse first waypoint to determine initial movement direction
        first_cmd = stm32_commands[0]
        values = first_cmd.replace('go_to(', '').replace(')', '').split(',')
        first_waypoint_x = float(values[0].strip())
        first_waypoint_y = float(values[1].strip())
        
        # Calculate direction vector from robot to first waypoint
        dx = first_waypoint_x - robot_pose.x * 1000  # Convert to mm
        dy = first_waypoint_y - robot_pose.y * 1000
        
        # Calculate angle to first waypoint
        target_direction = math.degrees(math.atan2(dy, dx))
        
        # Get robot's current orientation
        robot_theta_deg = math.degrees(robot_pose.z) if robot_pose.z < 6.28 else robot_pose.z
        
        # Calculate how much the robot needs to rotate
        angle_to_waypoint = circular_angle_difference(robot_theta_deg, target_direction)
        
        # Determine which device requires less rotation:
        # - Front picker: efficient when robot is facing toward path (angle < 90°)
        # - Back picker: efficient when robot is facing away from path (angle > 90°)
        prefer_back = angle_to_waypoint > 90
        
        # Check availability with preference
        front_available = self.is_front_available()
        back_available = self.is_back_available()
        
        if not front_available and not back_available:
            self.strategy.get_logger().warning("⚠️ Both front and back pickers are full!")
            return None
        
        # Apply preference based on path direction
        if prefer_back:
            if back_available:
                self.strategy.get_logger().info(
                    f'🎯 Back picker selected (path angle={angle_to_waypoint:.1f}° > 90°, '
                    f'robot={robot_theta_deg:.1f}°, path_dir={target_direction:.1f}°) - backwards motion optimal'
                )
                return 'ESP_back'
            elif front_available:
                self.strategy.get_logger().info(
                    f'🔄 Front picker selected (back preferred but unavailable, path angle={angle_to_waypoint:.1f}°)'
                )
                return 'ESP_front'
        else:
            if front_available:
                self.strategy.get_logger().info(
                    f'🎯 Front picker selected (path angle={angle_to_waypoint:.1f}° ≤ 90°, '
                    f'robot={robot_theta_deg:.1f}°, path_dir={target_direction:.1f}°) - forward motion optimal'
                )
                return 'ESP_front'
            elif back_available:
                self.strategy.get_logger().info(
                    f'🔄 Back picker selected (front preferred but unavailable, path angle={angle_to_waypoint:.1f}°)'
                )
                return 'ESP_back'
        
        return None
    
    def update_available_batches(self):
        """Update list of available batches from pathfinder/map"""
        try:
            # Get active batches from pathfinder service
            batch_info = self.pathfinder.get_active_batches()
            
            if batch_info is None:
                self.strategy.get_logger().warning('⚠️ Failed to get active batches from pathfinder')
                return
            
            # Convert arrays to dictionary indexed by known_id
            self.available_batches.clear()
            
            for i in range(len(batch_info['known_ids'])):
                known_id = batch_info['known_ids'][i]
                self.available_batches[known_id] = {
                    'known_id': known_id,
                    'batch_type': batch_info['batch_types'][i],
                    'member_count': batch_info['member_counts'][i],
                    'direction': batch_info['directions'][i],
                    'center': batch_info['centers'][i],
                    'quality': batch_info['qualities'][i]
                }
            
            self.last_batch_update = time.time()
            
            fx_count = sum(1 for b in self.available_batches.values() if b['batch_type'] == 'fx')
            sx_count = sum(1 for b in self.available_batches.values() if b['batch_type'] == 'sx')
            self.strategy.get_logger().info(
                f'🧠 Batch list updated: {len(self.available_batches)} batches '
                f'(FX={fx_count}, SX={sx_count})'
            )
        
        except Exception as e:
            self.strategy.get_logger().error(f'❌ Error updating batch list: {e}')
    
    def get_pickup_poses(self, known_id: str) -> Optional[List[Dict]]:
        """
        Get pickup poses for a specific batch by known_id
        
        Args:
            known_id: Batch known_id (e.g., 'S1', 'S2', 'S1_WEB', etc.)
        
        Returns:
            List of pickup pose dictionaries or None if not found
        """
        # Strip _WEB suffix for simulation batches (S1_WEB → S1)
        base_id = known_id.upper().replace('_WEB', '')
        return PICKUP_POSES.get(base_id)
    
    def find_optimal_putdown_position(self, center_x: float, center_y: float, 
                                      reference_x: float = None, reference_y: float = None) -> Optional[Dict]:
        """
        Find optimal putdown position within 30cm radius circle of a type B zone.
        The putdown point is positioned to approach the center point.
        
        Args:
            center_x: Center x coordinate in mm
            center_y: Center y coordinate in mm
            reference_x: Reference x position (e.g., pickup position) in mm, or None to use robot pose
            reference_y: Reference y position (e.g., pickup position) in mm, or None to use robot pose
        
        Returns:
            Dictionary with 'x', 'y', 'theta' for putdown position, or None if no reference available
        """
        # Use provided reference position, otherwise fall back to robot pose
        if reference_x is not None and reference_y is not None:
            robot_x_mm = reference_x
            robot_y_mm = reference_y
            self.strategy.get_logger().info(
                f'📍 Using reference position: ({robot_x_mm:.0f}mm, {robot_y_mm:.0f}mm)'
            )
        else:
            robot_pose = self.strategy.latest_robot_pose
            if robot_pose is None:
                self.strategy.get_logger().warning('⚠️ Robot pose not available for putdown optimization')
                return None
            robot_x_mm = robot_pose.x * 1000
            robot_y_mm = robot_pose.y * 1000
        
        # Calculate vector from center to robot
        dx = robot_x_mm - center_x
        dy = robot_y_mm - center_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # If robot is very close to center, use a default position
        if distance < 50:  # Less than 5cm
            self.strategy.get_logger().warning('⚠️ Robot too close to center, using default offset')
            dx = 300
            dy = 0
            distance = 300
        
        # Normalize direction vector
        dx_norm = dx / distance
        dy_norm = dy / distance
        
        # Place putdown point 30cm (300mm) from center toward robot's direction
        putdown_x = center_x + dx_norm * 300
        putdown_y = center_y + dy_norm * 300
        
        # Calculate angle pointing toward center
        angle_to_center = math.degrees(math.atan2(-dy_norm, -dx_norm))
        
        self.strategy.get_logger().info(
            f'📍 Optimal putdown: ({putdown_x:.0f}mm, {putdown_y:.0f}mm, {angle_to_center:.1f}°) '
            f'from center ({center_x:.0f}mm, {center_y:.0f}mm)'
        )
        
        return {
            'x': putdown_x,
            'y': putdown_y,
            'theta': angle_to_center
        }
    
    def choose_best_pickup_pose(self, known_id: str) -> Optional[Dict]:
        """
        Choose the best pickup pose for a batch based on robot position
        
        Args:
            known_id: Batch known_id
        
        Returns:
            Best pickup pose dict with 'x', 'y', 'theta' keys, or None if not found
        """
        poses = self.get_pickup_poses(known_id)
        if not poses:
            self.strategy.get_logger().warning(f'⚠️ No pickup poses found for {known_id}')
            return None
        
        # If only one pose, return it
        if len(poses) == 1:
            return poses[0]
        
        # Get current robot position
        robot_pose = self.strategy.latest_robot_pose
        if robot_pose is None:
            self.strategy.get_logger().warning('⚠️ Robot pose not available, using first pose')
            return poses[0]
        
        # Calculate distance to each pose and return closest
        robot_x_mm = robot_pose.x * 1000
        robot_y_mm = robot_pose.y * 1000
        
        def distance(pose):
            dx = pose['x'] - robot_x_mm
            dy = pose['y'] - robot_y_mm
            return math.sqrt(dx*dx + dy*dy)
        
        best_pose = min(poses, key=distance)
        self.strategy.get_logger().info(
            f'📍 Chose pickup pose for {known_id}: x={best_pose["x"]}, y={best_pose["y"]} '
            f'(distance: {distance(best_pose):.0f}mm)'
        )
        return best_pose
    
    def choose_best_putdown_zone(self, reference_x: float = None, reference_y: float = None) -> Optional[Tuple[str, Dict]]:
        """
        Choose the best putdown zone from available type B zones based on proximity.
        Type A zones are skipped for now.
        
        Args:
            reference_x: Reference x position (e.g., pickup position) in mm, or None to use robot pose
            reference_y: Reference y position (e.g., pickup position) in mm, or None to use robot pose
        
        Returns:
            Tuple of (zone_name, zone_dict) with zone information, or None if no reference available
        """
        # Use provided reference position, otherwise fall back to robot pose
        if reference_x is not None and reference_y is not None:
            robot_x_mm = reference_x
            robot_y_mm = reference_y
            self.strategy.get_logger().info(
                f'🎯 Selecting putdown zone from reference: ({robot_x_mm:.0f}mm, {robot_y_mm:.0f}mm)'
            )
        else:
            robot_pose = self.strategy.latest_robot_pose
            if robot_pose is None:
                self.strategy.get_logger().warning('⚠️ Robot pose not available for putdown zone selection')
                return None
            robot_x_mm = robot_pose.x * 1000
            robot_y_mm = robot_pose.y * 1000
        
        # Filter for type B zones only
        type_b_zones = {name: zone for name, zone in PUTDOWN_POSES.items() if zone['type'] == 'B'}
        
        if not type_b_zones:
            self.strategy.get_logger().warning('⚠️ No type B putdown zones available')
            return None
        
        # Find closest type B zone
        best_zone = None
        best_distance = float('inf')
        best_name = None
        
        for zone_name, zone in type_b_zones.items():
            dx = zone['x'] - robot_x_mm
            dy = zone['y'] - robot_y_mm
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < best_distance:
                best_distance = distance
                best_zone = zone
                best_name = zone_name
        
        self.strategy.get_logger().info(
            f'🎯 Selected putdown zone: {best_name} at ({best_zone["x"]}mm, {best_zone["y"]}mm) '
            f'- distance: {best_distance:.0f}mm'
        )
        
        return (best_name, best_zone)
    
    def get_optimal_batch(self, batch_type: Optional[str] = None, 
                          min_members: int = 1) -> Optional[Dict]:
        """
        Find the optimal batch to steal based on robot position and batch properties
        
        Args:
            batch_type: Filter by batch type ('fx', 'sx'), or None for all
            min_members: Minimum number of members required
        
        Returns:
            Dictionary with batch info, pickup_pose, distance, and score, or None if no batches
        """
        # Update batch list
        self.update_available_batches()
        
        # Get current robot position
        robot_pose = self.strategy.latest_robot_pose
        if robot_pose is None:
            self.strategy.get_logger().warning('⚠️ Cannot determine optimal batch: robot pose not available')
            return None
        
        robot_x_mm = robot_pose.x * 1000
        robot_y_mm = robot_pose.y * 1000

        enemy_pos_mm = self._get_enemy_position_mm()
        if enemy_pos_mm is not None:
            self.strategy.get_logger().info(
                f'⚠️ Enemy considered in scoring: hard_block_radius={self.hard_block_radius_mm:.0f}mm, '
                f'near_penalty_radius={self.near_penalty_radius_mm:.0f}mm'
            )
        
        # Filter batches
        batches = list(self.available_batches.values())
        
        if batch_type:
            batches = [b for b in batches if b['batch_type'] == batch_type]
        
        batches = [b for b in batches if b['member_count'] >= min_members]
        
        # Filter batches that have pickup poses (mainly SX batches)
        batches_with_poses = []
        for batch in batches:
            known_id = batch['known_id']
            pickup_pose = self.choose_best_pickup_pose(known_id)
            if pickup_pose:
                batches_with_poses.append({
                    'batch': batch,
                    'pickup_pose': pickup_pose
                })
        
        if not batches_with_poses:
            self.strategy.get_logger().info('ℹ️ No batches available with pickup poses')
            return None
        
        # Score each batch
        scored_batches = []
        
        for item in batches_with_poses:
            batch = item['batch']
            pickup_pose = item['pickup_pose']

            # Hard reject if enemy blocks this batch/pickup area
            if enemy_pos_mm is not None:
                enemy_x_mm, enemy_y_mm = enemy_pos_mm
                if self._is_hard_blocked_by_enemy(batch, pickup_pose, enemy_x_mm, enemy_y_mm):
                    _, _, min_enemy_dist = self._get_batch_enemy_distances_mm(
                        batch, pickup_pose, enemy_x_mm, enemy_y_mm
                    )
                    self.strategy.get_logger().info(
                        f'🚫 Skipping batch {batch["known_id"]}: enemy too close '
                        f'({min_enemy_dist:.0f}mm <= {self.hard_block_radius_mm:.0f}mm)'
                    )
                    continue
            
            # Calculate distance to pickup point
            dx = pickup_pose['x'] - robot_x_mm
            dy = pickup_pose['y'] - robot_y_mm
            distance_mm = math.sqrt(dx*dx + dy*dy)
            
            # Calculate score (closer is better, more members is better)
            # Normalize factors
            max_distance = 3000  # mm
            distance_score = max(0, 1 - (distance_mm / max_distance))
            
            max_members = 6
            member_score = min(1.0, batch['member_count'] / max_members)
            
            quality_score = batch['quality'] / 100.0
            
            # Weighted total score
            total_score = (
                distance_score * 1.0 +
                member_score * 2.0 +
                quality_score * 1.5
            )

            enemy_penalty = 0.0
            if enemy_pos_mm is not None:
                enemy_x_mm, enemy_y_mm = enemy_pos_mm
                enemy_penalty = self._compute_enemy_penalty(batch, pickup_pose, enemy_x_mm, enemy_y_mm)
                total_score -= enemy_penalty
            
            scored_batches.append({
                'known_id': batch['known_id'],
                'batch_type': batch['batch_type'],
                'member_count': batch['member_count'],
                'direction': batch['direction'],
                'center': batch['center'],
                'quality': batch['quality'],
                'pickup_pose': pickup_pose,
                'distance_mm': distance_mm,
                'score': total_score,
                'enemy_penalty': enemy_penalty
            })

        if not scored_batches:
            self.strategy.get_logger().info('ℹ️ No valid batches left after enemy blocking filter')
            return None
        
        # Sort by score (highest first)
        scored_batches.sort(key=lambda x: x['score'], reverse=True)
        
        best = scored_batches[0]
        self.strategy.get_logger().info(
            f'🎯 Optimal batch: {best["known_id"]} (type={best["batch_type"]}, '
            f'members={best["member_count"]}, distance={best["distance_mm"]:.0f}mm, '
            f'enemy_penalty={best.get("enemy_penalty", 0.0):.2f}, score={best["score"]:.2f})'
        )
        
        return best
    
    def steal_batch(self, batch_type: Optional[str] = None, 
                    device: Optional[str] = None,
                    min_members: int = 1) -> Optional[List[Dict]]:
        """
        Generate complete tasks to steal a batch: pickup task + putdown task
        
        Args:
            batch_type: Type of batch to steal ('fx', 'sx'), or None for any
            device: Device to use ('ESP_front' or 'ESP_back'), or None to auto-select
            min_members: Minimum number of members required
        
        Returns:
            List containing [pickup_task, putdown_task], or None if generation fails
        """
        self.strategy.get_logger().info(
            f'🎯 steal_batch() called - searching for optimal batch '
            f'(type={batch_type or "any"}, min_members={min_members})'
        )
        
        # Get optimal batch
        optimal_batch = self.get_optimal_batch(batch_type, min_members)
        
        if optimal_batch is None:
            self.strategy.get_logger().warning('⚠️ No batches available to steal')
            return None
        
        # Extract batch information
        known_id = optimal_batch['known_id']
        pickup_pose = optimal_batch['pickup_pose']
        
        # Convert pickup pose coordinates
        pickup_x_mm = pickup_pose['x']
        pickup_y_mm = pickup_pose['y']
        pickup_angles = pickup_pose['theta']
        
        # Choose angle closest to current robot orientation
        robot_pose = self.strategy.latest_robot_pose
        if robot_pose and len(pickup_angles) > 1:
            robot_theta_deg = math.degrees(robot_pose.z) if robot_pose.z < 6.28 else robot_pose.z
            # Use circular difference to handle wrap-around (e.g., -179° vs 179°)
            angle_diffs = [circular_angle_difference(robot_theta_deg, angle) for angle in pickup_angles]
            pickup_theta = pickup_angles[angle_diffs.index(min(angle_diffs))]
        else:
            pickup_theta = pickup_angles[0]
        
        self.strategy.get_logger().info(
            f'🎯 Selected batch {known_id}: '
            f'pos=({pickup_x_mm}mm, {pickup_y_mm}mm), '
            f'angle={pickup_theta}°, '
            f'members={optimal_batch["member_count"]}, '
            f'distance={optimal_batch["distance_mm"]:.0f}mm'
        )
        
        # Generate path using pathfinder BEFORE selecting device
        self.strategy.get_logger().info('🗺️  Generating path to batch...')
        
        # Get batch direction and center for proper approach positioning
        batch_direction = optimal_batch['direction']
        batch_center = optimal_batch['center']
        
        # Adjust goal position based on batch orientation (same logic as strategy_manager.handle_global_goal)
        # This ensures proper approach for horizontal/vertical batches
        pickup_x = pickup_x_mm / 1000.0  # Convert to meters
        pickup_y = pickup_y_mm / 1000.0
        
        if batch_direction == "horizontal" and pickup_y < batch_center.y:
            # Approaching from below a horizontal batch
            adjusted_y = pickup_y - 0.1
            adjusted_x = pickup_x
            self.strategy.get_logger().info(f'📐 Horizontal batch - approaching from below: y offset = -0.1m')
        elif batch_direction == "horizontal" and pickup_y > batch_center.y:
            # Approaching from above a horizontal batch
            adjusted_y = pickup_y + 0.1
            adjusted_x = pickup_x
            self.strategy.get_logger().info(f'📐 Horizontal batch - approaching from above: y offset = +0.1m')
        elif batch_direction == "vertical" and pickup_x < batch_center.x:
            # Approaching from left of a vertical batch
            adjusted_x = pickup_x - 0.1
            adjusted_y = pickup_y
            self.strategy.get_logger().info(f'📐 Vertical batch - approaching from left: x offset = -0.1m')
        elif batch_direction == "vertical" and pickup_x > batch_center.x:
            # Approaching from right of a vertical batch
            adjusted_x = pickup_x + 0.1
            adjusted_y = pickup_y
            self.strategy.get_logger().info(f'📐 Vertical batch - approaching from right: x offset = +0.1m')
        else:
            # Default case: no adjustment needed
            adjusted_x = pickup_x
            adjusted_y = pickup_y
            self.strategy.get_logger().warning(
                f'⚠️ Edge case: position equals batch center or unexpected direction "{batch_direction}"'
            )
        
        # Set goal for pathfinder with adjusted position
        # Convert pickup_theta from degrees to radians (pathfinder expects radians)
        pickup_theta_rad = math.radians(pickup_theta)
        self.pathfinder.set_goal(adjusted_x, adjusted_y, pickup_theta_rad)
        
        # Get path waypoints from pathfinder
        stm32_commands, path_success = self.pathfinder.path()
        
        if not path_success:
            self.strategy.get_logger().error(f'❌ Failed to generate path to {known_id}')
            return None
        
        self.strategy.get_logger().info(f'✅ Path generated: {len(stm32_commands)} waypoints')
        
        # NOW select device based on actual path trajectory (if not manually specified)
        if device is None:
            device = self.get_optimal_device_from_path(stm32_commands)
            if device is None:
                self.strategy.get_logger().error('❌ No available device (both front and back are full)')
                return None
            self.strategy.get_logger().info(f'🔧 Selected device: {device}')
        
        # Build task sequence with STM32 waypoints + ESP command (like static YAML tasks)
        task_sequence = []
        
        # Add STM32 waypoint commands
        last_cmd = None
        for cmd in stm32_commands:
            # Parse waypoint: "go_to(x, y, speed)"
            values = cmd.replace('go_to(', '').replace(')', '').split(',')
            x = float(values[0].strip())
            y = float(values[1].strip())
            speed = int(values[2].strip())
            
            # Avoid duplicate consecutive waypoints
            if cmd != last_cmd:
                # Choose command based on device (front=forward, back=backward)
                if device == 'ESP_back':
                    stm32_cmd = 'go_to_backwards'
                else:
                    stm32_cmd = 'go_to'
                
                task_sequence.append({
                    'device': 'STM',
                    'command': stm32_cmd,
                    'params': [x, y, 1100.0, {'from_pathfinding': True}],
                    'wait': False
                })
            last_cmd = cmd
        
        # Add final orientation command if goal angle is available
        goal_angle = self.pathfinder.goal_angle
        if goal_angle is not None:
            # Add final movement to pickup position (like strategy_manager path() logic)
            # This ensures proper alignment after orientation
            if device == 'ESP_front':
                task_sequence.append({
                    'device': 'STM',
                    'command': 'go_to',
                    'params': [pickup_x_mm, pickup_y_mm, 1100.0, 1.0, {'from_pathfinding': True}],
                    'wait': False
                })
                self.strategy.get_logger().info(
                    f'📍 Added final go_to after orientate: x={pickup_x_mm:.0f}mm, y={pickup_y_mm:.0f}mm'
                )
            elif device == 'ESP_back':
                task_sequence.append({
                    'device': 'STM',
                    'command': 'go_to_backwards',
                    'params': [pickup_x_mm, pickup_y_mm, 1100.0, 1.0, {'from_pathfinding': True}],
                    'wait': False
                })
                self.strategy.get_logger().info(
                    f'📍 Added final go_to_backwards after orientate: x={pickup_x_mm:.0f}mm, y={pickup_y_mm:.0f}mm'
                )
        
        # Add ESP pick_up command at the end
        task_sequence.append({
            'device': device,
            'command': 'pick_up',
            'params': {},
            'wait': True  # Wait for STM32 to reach position before picking
        })
        
        self.strategy.get_logger().info(
            f'📋 Task sequence for {device}: {len([c for c in task_sequence if c["device"] == "STM"])} STM32 commands + '
            f'1 {device} pickup command'
        )
        
        # Create pickup task structure (compatible with load_task_to_queues)
        pickup_task = {
            'name': f'steal_{known_id}_{device.split("_")[1]}',
            'sequence': task_sequence,
            'global_goal': {
                'x': pickup_x_mm,
                'y': pickup_y_mm,
                'theta': pickup_theta
            }
        }
        
        # NOTE: Picker status should be updated by the executor after successful pickup,
        # not here during task generation. Premature status updates can cause stale state
        # if the task fails or is interrupted.
        
        self.strategy.get_logger().info(f'✅ Generated pickup task: {pickup_task["name"]}')

        # Exclude picked batch from obstacle set for subsequent path generation (e.g., putdown path)
        # while keeping its metadata queryable by known_id.
        if hasattr(self.pathfinder, 'add_untracked_batch'):
            self.pathfinder.add_untracked_batch(known_id)
        
        # ========== NOW CREATE PUTDOWN TASK ==========
        
        # Select best putdown zone (type B only for now) based on pickup position
        putdown_zone_result = self.choose_best_putdown_zone(
            reference_x=pickup_x_mm,
            reference_y=pickup_y_mm
        )
        if putdown_zone_result is None:
            self.strategy.get_logger().warning('⚠️ No putdown zone available, returning pickup task only')
            if hasattr(self.pathfinder, 'remove_untracked_batch'):
                self.pathfinder.remove_untracked_batch(known_id)
            return [pickup_task]
        
        zone_name, zone = putdown_zone_result
        
        # Calculate optimal putdown position within 30cm radius circle
        # Use pickup position as reference (robot will be there after pickup)
        putdown_pos = self.find_optimal_putdown_position(
            zone['x'], 
            zone['y'],
            reference_x=pickup_x_mm,
            reference_y=pickup_y_mm
        )
        if putdown_pos is None:
            self.strategy.get_logger().warning('⚠️ Could not calculate putdown position, returning pickup task only')
            if hasattr(self.pathfinder, 'remove_untracked_batch'):
                self.pathfinder.remove_untracked_batch(known_id)
            return [pickup_task]
        
        # Create putdown task using the same device
        putdown_task = self.create_put_down_task(
            device=device,
            target_x=putdown_pos['x'],
            target_y=putdown_pos['y'],
            target_theta=putdown_pos['theta'],
            start_x=pickup_x_mm,
            start_y=pickup_y_mm,
            start_theta=pickup_theta
        )
        
        if putdown_task is None:
            self.strategy.get_logger().warning('⚠️ Failed to generate putdown task, returning pickup task only')
            if hasattr(self.pathfinder, 'remove_untracked_batch'):
                self.pathfinder.remove_untracked_batch(known_id)
            return [pickup_task]

        # Putdown task is now generated. Restore default obstacle tracking for future planning calls.
        if hasattr(self.pathfinder, 'remove_untracked_batch'):
            self.pathfinder.remove_untracked_batch(known_id)
        
        self.strategy.get_logger().info(f'✅ Generated complete steal sequence: pickup + putdown at {zone_name}')
        return [pickup_task, putdown_task]
    
    def create_put_down_task(self, device: str, target_x: float, target_y: float,
                            target_theta: float, start_x: Optional[float] = None,
                            start_y: Optional[float] = None, start_theta: Optional[float] = None) -> Optional[Dict]:
        """
        Create a put_down task to drop batch at target location
        
        Args:
            device: Device that has the batch ('ESP_front' or 'ESP_back')
            target_x: Target x coordinate in mm
            target_y: Target y coordinate in mm
            target_theta: Target orientation in degrees
            start_x: Optional start x in mm (if provided, overrides live robot pose)
            start_y: Optional start y in mm (if provided, overrides live robot pose)
            start_theta: Optional start theta in degrees (if provided, overrides live robot pose)
        
        Returns:
            Task dictionary for putting down batch, or None if path generation fails
        """
        self.strategy.get_logger().info(
            f'📦 Creating put_down task: device={device}, '
            f'target=({target_x}mm, {target_y}mm, {target_theta}°)'
        )
        
        # Convert theta from degrees to radians for pathfinder
        target_theta_rad = math.radians(target_theta)
        
        # Generate path using pathfinder
        self.pathfinder.set_goal(
            target_x / 1000.0,
            target_y / 1000.0,
            target_theta_rad
        )
        
        # Get path waypoints (use explicit start pose if provided)
        if start_x is not None and start_y is not None and start_theta is not None:
            start_theta_rad = math.radians(start_theta)
            stm32_commands, path_success = self.pathfinder.path(
                start_x=start_x / 1000.0,
                start_y=start_y / 1000.0,
                start_theta=start_theta_rad
            )
        else:
            stm32_commands, path_success = self.pathfinder.path()
        
        if not path_success:
            self.strategy.get_logger().error('❌ Failed to generate path for put_down')
            return None
        
        self.strategy.get_logger().info(f'✅ Path generated: {len(stm32_commands)} waypoints')
        
        # Build task sequence with STM32 waypoints + ESP command
        task_sequence = []
        
        # Add STM32 waypoint commands
        last_cmd = None
        first_stm_command = True
        for cmd in stm32_commands:
            values = cmd.replace('go_to(', '').replace(')', '').split(',')
            x = float(values[0].strip())
            y = float(values[1].strip())
            speed = int(values[2].strip())
            
            if cmd != last_cmd:
                if device == 'ESP_back':
                    stm32_cmd = 'go_to_backwards'
                else:
                    stm32_cmd = 'go_to'
                
                task_sequence.append({
                    'device': 'STM',
                    'command': stm32_cmd,
                    'params': [x, y, 1100.0, {'from_pathfinding': True}],
                    'wait': first_stm_command
                })
                first_stm_command = False
            last_cmd = cmd
        
        # Add final orientation
        goal_angle = self.pathfinder.goal_angle
        if goal_angle is not None:
            # # Convert from radians to degrees
            goal_angle_deg = math.degrees(goal_angle)
            task_sequence.append({
                'device': 'STM',
                'command': 'orientate',
                'params': [goal_angle_deg, 0.0],  # [angle_in_degrees, is_radian=False]
                'wait': False
            })
        
        # Add ESP put_down command
        task_sequence.append({
            'device': device,
            'command': 'put_down',
            'params': {},
            'wait': True
        })
        
        task = {
            'name': f'putdown_{device.split("_")[1]}',
            'sequence': task_sequence,
            'global_goal': {
                'x': target_x,
                'y': target_y,
                'theta': target_theta
            }
        }
        
        # NOTE: Picker status should be updated by the executor after successful put_down,
        # not here during task generation. Premature status updates can cause stale state
        # if the task fails or is interrupted.
        
        self.strategy.get_logger().info(f'✅ Generated put_down task: {task["name"]}')
        return task


# Example usage for testing
if __name__ == '__main__':
    print("DecisionMaker module - use with StrategyManager")
