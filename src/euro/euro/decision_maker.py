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
from typing import Dict, List, Optional, Tuple
from geometry_msgs.msg import Point
import time


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
        
        self.strategy.get_logger().info('🧠 Decision Maker initialized')
    
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
        Get an available device (front or back) for pickup
        
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
            known_id: Batch known_id (e.g., 'S1', 'S2', etc.)
        
        Returns:
            List of pickup pose dictionaries or None if not found
        """
        return PICKUP_POSES.get(known_id)
    
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
            
            scored_batches.append({
                'known_id': batch['known_id'],
                'batch_type': batch['batch_type'],
                'member_count': batch['member_count'],
                'direction': batch['direction'],
                'center': batch['center'],
                'quality': batch['quality'],
                'pickup_pose': pickup_pose,
                'distance_mm': distance_mm,
                'score': total_score
            })
        
        # Sort by score (highest first)
        scored_batches.sort(key=lambda x: x['score'], reverse=True)
        
        best = scored_batches[0]
        self.strategy.get_logger().info(
            f'🎯 Optimal batch: {best["known_id"]} (type={best["batch_type"]}, '
            f'members={best["member_count"]}, distance={best["distance_mm"]:.0f}mm, '
            f'score={best["score"]:.2f})'
        )
        
        return best
    
    def steal_batch(self, batch_type: Optional[str] = None, 
                    device: Optional[str] = None,
                    min_members: int = 1) -> Optional[Dict]:
        """
        Generate a complete task to steal a batch (like static YAML tasks)
        
        Args:
            batch_type: Type of batch to steal ('fx', 'sx'), or None for any
            device: Device to use ('ESP_front' or 'ESP_back'), or None to auto-select
            min_members: Minimum number of members required
        
        Returns:
            Task dictionary compatible with load_task_to_queues, or None if no batch available
        """
        self.strategy.get_logger().info(
            f'🎯 steal_batch() called - searching for optimal batch '
            f'(type={batch_type or "any"}, min_members={min_members})'
        )
        
        # Auto-select device if not specified
        if device is None:
            device = self.get_available_device()
            if device is None:
                self.strategy.get_logger().error('❌ No available device (both front and back are full)')
                return None
        
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
            angle_diffs = [abs(robot_theta_deg - angle) for angle in pickup_angles]
            pickup_theta = pickup_angles[angle_diffs.index(min(angle_diffs))]
        else:
            pickup_theta = pickup_angles[0]
        
        self.strategy.get_logger().info(
            f'🎯 Selected batch {known_id}: '
            f'pos=({pickup_x_mm}mm, {pickup_y_mm}mm), '
            f'angle={pickup_theta}°, '
            f'members={optimal_batch["member_count"]}, '
            f'distance={optimal_batch["distance_mm"]:.0f}mm, '
            f'device={device}'
        )
        
        # Generate path using pathfinder (like path() in strategy_manager)
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
        self.pathfinder.set_goal(adjusted_x, adjusted_y, pickup_theta)
        
        # Get path waypoints from pathfinder
        stm32_commands, path_success = self.pathfinder.path()
        
        if not path_success:
            self.strategy.get_logger().error(f'❌ Failed to generate path to {known_id}')
            return None
        
        self.strategy.get_logger().info(f'✅ Path generated: {len(stm32_commands)} waypoints')
        
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
            task_sequence.append({
                'device': 'STM',
                'command': 'orientate',
                'params': [goal_angle, 0.0],  # [angle, is_radian]
                'wait': False
            })
            
            # Add final movement to pickup position (like strategy_manager path() logic)
            # This ensures proper alignment after orientation
            if device == 'ESP_front':
                task_sequence.append({
                    'device': 'STM',
                    'command': 'go_to',
                    'params': [pickup_x_mm / 1000.0, pickup_y_mm / 1000.0, 1100.0, 1.0, {'from_pathfinding': True}],
                    'wait': False
                })
                self.strategy.get_logger().info(
                    f'📍 Added final go_to after orientate: x={pickup_x_mm/1000.0:.3f}m, y={pickup_y_mm/1000.0:.3f}m'
                )
            elif device == 'ESP_back':
                task_sequence.append({
                    'device': 'STM',
                    'command': 'go_to_backwards',
                    'params': [pickup_x_mm / 1000.0, pickup_y_mm / 1000.0, 1100.0, 1.0, {'from_pathfinding': True}],
                    'wait': False
                })
                self.strategy.get_logger().info(
                    f'📍 Added final go_to_backwards after orientate: x={pickup_x_mm/1000.0:.3f}m, y={pickup_y_mm/1000.0:.3f}m'
                )
        
        # Add ESP pick_up command at the end
        task_sequence.append({
            'device': device,
            'command': 'pick_up',
            'params': {},
            'wait': True  # Wait for STM32 to reach position before picking
        })
        
        self.strategy.get_logger().info(
            f'📋 Task sequence: {len([c for c in task_sequence if c["device"] == "STM"])} STM32 commands + '
            f'1 {device} command'
        )
        
        # Create task structure (compatible with load_task_to_queues)
        task = {
            'name': f'steal_{known_id}_{device.split("_")[1]}',
            'sequence': task_sequence,
            'global_goal': {
                'x': pickup_x_mm,
                'y': pickup_y_mm,
                'theta': pickup_theta
            }
        }
        
        # Mark device as full after pickup
        if device == 'ESP_front':
            self.set_front_status(True)
        elif device == 'ESP_back':
            self.set_back_status(True)
        
        self.strategy.get_logger().info(f'✅ Generated steal task: {task["name"]}')
        return task
    
    def create_put_down_task(self, device: str, target_x: float, target_y: float, 
                            target_theta: float) -> Optional[Dict]:
        """
        Create a put_down task to drop batch at target location
        
        Args:
            device: Device that has the batch ('ESP_front' or 'ESP_back')
            target_x: Target x coordinate in mm
            target_y: Target y coordinate in mm
            target_theta: Target orientation in degrees
        
        Returns:
            Task dictionary for putting down batch, or None if path generation fails
        """
        self.strategy.get_logger().info(
            f'📦 Creating put_down task: device={device}, '
            f'target=({target_x}mm, {target_y}mm, {target_theta}°)'
        )
        
        # Generate path using pathfinder
        self.pathfinder.set_goal(
            target_x / 1000.0,
            target_y / 1000.0,
            target_theta
        )
        
        # Get path waypoints
        stm32_commands, path_success = self.pathfinder.path()
        
        if not path_success:
            self.strategy.get_logger().error('❌ Failed to generate path for put_down')
            return None
        
        self.strategy.get_logger().info(f'✅ Path generated: {len(stm32_commands)} waypoints')
        
        # Build task sequence with STM32 waypoints + ESP command
        task_sequence = []
        
        # Add STM32 waypoint commands
        last_cmd = None
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
                    'wait': False
                })
            last_cmd = cmd
        
        # Add final orientation
        goal_angle = self.pathfinder.goal_angle
        if goal_angle is not None:
            task_sequence.append({
                'device': 'STM',
                'command': 'orientate',
                'params': [goal_angle, 0.0],
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
        
        # Mark device as empty after put_down
        if device == 'ESP_front':
            self.set_front_status(False)
        elif device == 'ESP_back':
            self.set_back_status(False)
        
        self.strategy.get_logger().info(f'✅ Generated put_down task: {task["name"]}')
        return task


# Example usage for testing
if __name__ == '__main__':
    print("DecisionMaker module - use with StrategyManager")
