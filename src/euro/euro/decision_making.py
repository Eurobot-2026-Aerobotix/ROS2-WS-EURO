#!/usr/bin/env python3
"""
Decision Making Module for Eurobot Strategy

This module periodically queries available batches from the pathfinder
and stores them for use by task generation and strategic decision making.

Responsibilities:
- Query active batches every 1 second from /pathfinder/get_active_batches service
- Store batch information (known_id, batch_type, member_count, direction, center, quality)
- Provide cached batch data to strategy manager for decision making
- Filter and score batches based on strategic criteria
"""

import rclpy
from rclpy.node import Node
from threading import Lock
from typing import Dict, List, Optional
import time

from geometry_msgs.msg import Point, Vector3, PoseStamped


class DecisionMaker:
    """
    Decision maker that maintains up-to-date batch information
    and provides it to the strategy manager
    """
    
    def __init__(self, node: Node, pathfinder_client):
        """
        Initialize the decision maker
        
        Args:
            node: ROS2 node instance (for logging and timers)
            pathfinder_client: PathfinderClient instance to call get_active_batches service
        """
        self.node = node
        self.pathfinder_client = pathfinder_client
        
        # Thread-safe storage for batch information
        self.batch_data: Dict[str, dict] = {}
        self.batch_data_lock = Lock()
        
        # Raw batch arrays from service (for direct access if needed)
        self.raw_batch_info: Optional[Dict[str, list]] = None
        
        # Timestamp of last successful update
        self.last_update_time = 0.0
        
        # Robot pose (from /robot_pose_raw topic - Vector3: x, y, z=theta)
        self.robot_pose_x = None
        self.robot_pose_y = None
        self.robot_pose_theta = None
        self.robot_pose_lock = Lock()
        
        # Enemy obstacle position (from /obstacle_relative topic - PoseStamped)
        self.enemy_x = None
        self.enemy_y = None
        self.enemy_detected = False
        self.enemy_lock = Lock()
        
        # Subscribe to robot pose
        self.robot_pose_subscription = node.create_subscription(
            Vector3,
            '/robot_pose_raw',
            self._robot_pose_callback,
            10
        )
        
        # Subscribe to enemy obstacle position
        self.enemy_subscription = node.create_subscription(
            PoseStamped,
            '/obstacle_relative',
            self._enemy_callback,
            10
        )
        
        # Create timer to update batch information every 1 second
        self.update_timer = node.create_timer(1.0, self._update_batch_info)
        
        self.node.get_logger().info('🧠 Decision Maker initialized - querying batches every 1s')
    
    def _update_batch_info(self):
        """Timer callback: Query active batches from pathfinder and update cache"""
        try:
            # Call service to get active batches
            batch_info = self.pathfinder_client.get_active_batches()
            
            if batch_info is None:
                self.node.get_logger().warning('⚠️  Failed to get active batches from pathfinder')
                return
            
            # Store raw batch info
            self.raw_batch_info = batch_info
            
            # Convert arrays to dictionary indexed by known_id for easier lookup
            with self.batch_data_lock:
                self.batch_data.clear()
                
                for i in range(len(batch_info['known_ids'])):
                    known_id = batch_info['known_ids'][i]
                    self.batch_data[known_id] = {
                        'known_id': known_id,
                        'batch_type': batch_info['batch_types'][i],
                        'member_count': batch_info['member_counts'][i],
                        'direction': batch_info['directions'][i],
                        'center': batch_info['centers'][i],
                        'quality': batch_info['qualities'][i]
                    }
                
                self.last_update_time = time.time()
            
            # Log summary (only on changes or every 10 updates)
            if not hasattr(self, '_update_count'):
                self._update_count = 0
            self._update_count += 1
            
            if self._update_count % 10 == 1:  # Log every 10 updates
                fx_count = sum(1 for b in self.batch_data.values() if b['batch_type'] == 'fx')
                sx_count = sum(1 for b in self.batch_data.values() if b['batch_type'] == 'sx')
                self.node.get_logger().info(
                    f'🧠 Batch cache updated: {len(self.batch_data)} batches '
                    f'(FX={fx_count}, SX={sx_count})'
                )
        
        except Exception as e:
            self.node.get_logger().error(f'❌ Error updating batch info: {e}')
    
    def _robot_pose_callback(self, msg: Vector3):
        """Callback for /robot_pose_raw topic (Vector3: x, y, z=theta)"""
        with self.robot_pose_lock:
            self.robot_pose_x = msg.x
            self.robot_pose_y = msg.y
            self.robot_pose_theta = msg.z
    
    def _enemy_callback(self, msg: PoseStamped):
        """Callback for /obstacle_relative topic (enemy robot position)"""
        with self.enemy_lock:
            self.enemy_x = msg.pose.position.x
            self.enemy_y = msg.pose.position.y
            self.enemy_detected = True
    
    def get_robot_pose(self) -> Optional[Dict[str, float]]:
        """
        Get current robot pose
        
        Returns:
            Dictionary with 'x', 'y', 'theta' or None if not available
        """
        with self.robot_pose_lock:
            if self.robot_pose_x is None:
                return None
            return {
                'x': self.robot_pose_x,
                'y': self.robot_pose_y,
                'theta': self.robot_pose_theta
            }
    
    def get_enemy_position(self) -> Optional[Dict[str, float]]:
        """
        Get current enemy robot position
        
        Returns:
            Dictionary with 'x', 'y' or None if not detected
        """
        with self.enemy_lock:
            if not self.enemy_detected or self.enemy_x is None:
                return None
            return {
                'x': self.enemy_x,
                'y': self.enemy_y
            }
    
    def is_enemy_detected(self) -> bool:
        """Check if enemy robot is currently detected"""
        with self.enemy_lock:
            return self.enemy_detected
    
    def clear_enemy(self):
        """Clear enemy detection (e.g., when enemy is no longer visible)"""
        with self.enemy_lock:
            self.enemy_detected = False
            self.enemy_x = None
            self.enemy_y = None
    
    def get_all_batches(self) -> Dict[str, dict]:
        """
        Get all cached batch information
        
        Returns:
            Dictionary mapping known_id -> batch info dict
            {
                'known_id': str,
                'batch_type': str,
                'member_count': int,
                'direction': str,
                'center': Point,
                'quality': int
            }
        """
        with self.batch_data_lock:
            return dict(self.batch_data)
    
    def get_batch(self, known_id: str) -> Optional[dict]:
        """
        Get information for a specific batch
        
        Args:
            known_id: The batch known_id to query
        
        Returns:
            Batch info dict or None if not found
        """
        with self.batch_data_lock:
            return self.batch_data.get(known_id)
    
    def get_batches_by_type(self, batch_type: str) -> List[dict]:
        """
        Get all batches of a specific type
        
        Args:
            batch_type: 'fx', 'sx', or 'gx'
        
        Returns:
            List of batch info dictionaries
        """
        with self.batch_data_lock:
            return [b for b in self.batch_data.values() if b['batch_type'] == batch_type]
    
    def get_available_fx_batches(self, min_members: int = 1) -> List[dict]:
        """
        Get available FX batches with at least min_members
        
        Args:
            min_members: Minimum number of members required
        
        Returns:
            List of FX batch info dictionaries sorted by member count (descending)
        """
        fx_batches = self.get_batches_by_type('fx')
        available = [b for b in fx_batches if b['member_count'] >= min_members]
        # Sort by member count (descending) and quality (descending)
        available.sort(key=lambda x: (x['member_count'], x['quality']), reverse=True)
        return available
    
    def get_available_sx_batches(self, min_members: int = 1) -> List[dict]:
        """
        Get available SX batches with at least min_members
        
        Args:
            min_members: Minimum number of members required
        
        Returns:
            List of SX batch info dictionaries sorted by member count (descending)
        """
        sx_batches = self.get_batches_by_type('sx')
        available = [b for b in sx_batches if b['member_count'] >= min_members]
        # Sort by member count (descending) and quality (descending)
        available.sort(key=lambda x: (x['member_count'], x['quality']), reverse=True)
        return available
    
    def get_closest_batch(self, robot_pos: Optional[Point] = None, batch_type: Optional[str] = None, 
                          min_members: int = 1) -> Optional[dict]:
        """
        Get the closest batch to robot position
        
        Args:
            robot_pos: Robot position (Point with x, y), or None to use current robot pose
            batch_type: Filter by batch type ('fx', 'sx', 'gx'), or None for all
            min_members: Minimum number of members required
        
        Returns:
            Closest batch info dict or None if no batches available
        """
        # Use current robot pose if not provided
        if robot_pos is None:
            pose = self.get_robot_pose()
            if pose is None:
                self.node.get_logger().warning('⚠️  Cannot find closest batch: robot pose not available')
                return None
            robot_pos = Point()
            robot_pos.x = pose['x']
            robot_pos.y = pose['y']
        
        with self.batch_data_lock:
            batches = list(self.batch_data.values())
        
        # Filter by type if specified
        if batch_type:
            batches = [b for b in batches if b['batch_type'] == batch_type]
        
        # Filter by member count
        batches = [b for b in batches if b['member_count'] >= min_members]
        
        if not batches:
            return None
        
        # Calculate distances and find closest
        def distance(batch):
            center = batch['center']
            dx = center.x - robot_pos.x
            dy = center.y - robot_pos.y
            return (dx*dx + dy*dy) ** 0.5
        
        closest = min(batches, key=distance)
        return closest
    
    def is_batch_available(self, known_id: str, min_members: int = 1) -> bool:
        """
        Check if a specific batch is available with minimum members
        
        Args:
            known_id: Batch known_id to check
            min_members: Minimum number of members required
        
        Returns:
            True if batch exists and has at least min_members
        """
        batch = self.get_batch(known_id)
        if batch is None:
            return False
        return batch['member_count'] >= min_members
    
    def get_cache_age(self) -> float:
        """
        Get age of cached data in seconds
        
        Returns:
            Time since last successful update
        """
        if self.last_update_time == 0.0:
            return float('inf')
        return time.time() - self.last_update_time
    
    def is_cache_fresh(self, max_age_sec: float = 2.0) -> bool:
        """
        Check if cached data is fresh
        
        Args:
            max_age_sec: Maximum acceptable age in seconds
        
        Returns:
            True if cache is fresh
        """
        return self.get_cache_age() <= max_age_sec
    
    def shutdown(self):
        """Clean up resources"""
        if hasattr(self, 'update_timer') and self.update_timer:
            self.update_timer.cancel()
        self.node.get_logger().info('🧠 Decision Maker shutdown')


# Example standalone usage (for testing)
def main(args=None):
    rclpy.init(args=args)
    
    # Import here to avoid circular dependency
    from euro.pathfinder_controller import PathfinderClient
    
    # Create node and pathfinder client
    node = Node('decision_maker_test')
    pathfinder_client = PathfinderClient()
    
    # Create decision maker
    decision_maker = DecisionMaker(node, pathfinder_client)
    
    try:
        # Test querying batches after a few updates
        time.sleep(3)
        
        all_batches = decision_maker.get_all_batches()
        node.get_logger().info(f'All batches: {len(all_batches)}')
        
        fx_batches = decision_maker.get_available_fx_batches()
        node.get_logger().info(f'Available FX batches: {len(fx_batches)}')
        
        sx_batches = decision_maker.get_available_sx_batches()
        node.get_logger().info(f'Available SX batches: {len(sx_batches)}')
        
        # Keep running
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass
    finally:
        decision_maker.shutdown()
        node.destroy_node()
        pathfinder_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
