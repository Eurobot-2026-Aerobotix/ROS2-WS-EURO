#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point,Vector3
from nav_msgs.msg import Path

from hybrid_astar.srv import ComputePath, GetBatchColorPattern, DeleteBatch, GetActiveBatches
from std_srvs.srv import Trigger

class PathfinderClient(Node):
    """
    Client node that requests path planning
    and batch color pattern from a pathfinder node
    """

    def __init__(self):
        super().__init__('pathfinder_client')

        # ---- Service clients ----
        self.path_client = self.create_client(
            ComputePath,
            '/compute_path'
        )

        self.batch_color_client = self.create_client(
            GetBatchColorPattern,
            '/get_batch_color_pattern'
        )
        
        self.delete_batch_client = self.create_client(
            DeleteBatch,
            '/delete_batch'
        )
        
        self.update_map_client = self.create_client(
            Trigger,
            '/update_map'
        )
        
        self.reset_stability_client = self.create_client(
            Trigger,
            '/reset_map'
        )
        
        self.clear_map_client = self.create_client(
            Trigger,
            '/clear_map'
        )
        
        self.get_active_batches_client = self.create_client(
            GetActiveBatches,
            '/pathfinder/get_active_batches'
        )

        # Don't block on wait_for_service - it will timeout if service isn't available
        self.get_logger().info('Waiting for /compute_path service...')
        if not self.path_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('⚠️  /compute_path service not available after 5 seconds')
        else:
            self.get_logger().info('✅ /compute_path service is ready')
        
        self.get_logger().info('Waiting for /get_batch_color_pattern service...')
        if not self.batch_color_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('⚠️  /get_batch_color_pattern service not available after 5 seconds')
        else:
            self.get_logger().info('✅ /get_batch_color_pattern service is ready')
        
        self.get_logger().info('Waiting for /delete_batch service...')
        if not self.delete_batch_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('⚠️  /delete_batch service not available after 5 seconds')
        else:
            self.get_logger().info('✅ /delete_batch service is ready')
        
        self.get_logger().info('Waiting for /update_map service...')
        if not self.update_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('⚠️  /update_map service not available after 5 seconds')
        else:
            self.get_logger().info('✅ /update_map service is ready')
        
        self.get_logger().info('Waiting for /reset_map service...')
        if not self.reset_stability_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('⚠️  /reset_map service not available after 5 seconds')
        else:
            self.get_logger().info('✅ /reset_map service is ready')
        
        self.get_logger().info('Waiting for /clear_map service...')
        if not self.clear_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('⚠️  /clear_map service not available after 5 seconds')
        else:
            self.get_logger().info('✅ /clear_map service is ready')

        # Goal storage
        self.goal_x = None
        self.goal_y = None
        self.goal_angle = None

        # Enemy/obstacle position storage
        self.enemy_x = None
        self.enemy_y = None

        # Robot pose storage
        self.latest_robot_pose_x = None
        self.latest_robot_pose_y = None
        self.latest_robot_pose_theta = None

        # Subscribe to robot pose
        self.robot_pose_subscription = self.create_subscription(
            Vector3,
            '/robot_pose_raw',
            self.robot_pose_callback,
            10
        )

        # Subscribe to enemy obstacle position
        self.enemy_subscription = self.create_subscription(
            PoseStamped,
            '/obstacle_relative',
            self.enemy_callback,
            10
        )

    # =========================================================
    # GOAL MANAGEMENT
    # =========================================================
    def set_goal(self, x: float, y: float, angle: float):
        """
        Set the goal position
        
        Args:
            x: X coordinate
            y: Y coordinate
            angle: Angle in radians
        """
        self.goal_x = x
        self.goal_y = y
        self.goal_angle = angle
        self.get_logger().info(f'🎯 Goal set: ({x:.2f}, {y:.2f}, {angle:.2f} rad)')

    def clear_goal(self):
        """
        Clear the goal position
        """
        self.goal_x = None
        self.goal_y = None
        self.goal_angle = None
        self.get_logger().info('🗑️  Goal cleared')

    def robot_pose_callback(self, msg):
        """Update robot pose from /robot_pose_raw topic"""
        self.latest_robot_pose_x = msg.x
        self.latest_robot_pose_y = msg.y
        self.latest_robot_pose_theta = msg.z

    def enemy_callback(self, msg):
        """Update enemy obstacle position from /obstacle_relative topic"""
        self.enemy_x = msg.pose.position.x
        self.enemy_y = msg.pose.position.y

    # =========================================================
    # PATH PLANNING
    # =========================================================
    def path(self):
        """
        Plan a path from current robot position to stored goal position
        Uses latest robot pose from /robot_pose_raw and stored goal from set_goal()
        
        Returns:
            stm32_commands (list[str]): List of formatted STM32 commands
            success (bool): Whether path planning succeeded
        """
        # Check if robot pose is available
        if self.latest_robot_pose_x is None or self.latest_robot_pose_y is None or self.latest_robot_pose_theta is None:
            self.get_logger().error("No robot pose received yet!")
            return [], False
        
        # Check if goal is set
        if self.goal_x is None or self.goal_y is None or self.goal_angle is None:
            self.get_logger().error("No goal set! Use set_goal(x, y, angle) first.")
            return [], False
        
        # Create start pose from latest robot position
        start_pose = PoseStamped()
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.header.frame_id = "map"
        start_pose.pose.position.x = self.latest_robot_pose_x
        start_pose.pose.position.y = self.latest_robot_pose_y
        start_pose.pose.position.z = 0.0
        start_pose.pose.orientation = self._yaw_to_quaternion(self.latest_robot_pose_theta)
        
        # Create goal pose from stored goal
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = self.goal_x
        goal_pose.pose.position.y = self.goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = self._yaw_to_quaternion(self.goal_angle)
        
        # Get path
        stm32_commands, success = self.get_path(start_pose, goal_pose)
        
        return stm32_commands, success
    
    def _yaw_to_quaternion(self, yaw: float):
        """Convert yaw angle to quaternion"""
        import math
        from geometry_msgs.msg import Quaternion
        
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

    # =========================================================
    # 1️⃣ GET PATH
    # =========================================================
    def get_path(self, start_pose: PoseStamped, goal_pose: PoseStamped):
        """
        Request a path from the path planning node

        Returns:
            stm32_commands (list[str])
            success (bool)
        """
        
        # Check if service is available
        if not self.path_client.service_is_ready():
            self.get_logger().error('❌ /compute_path service is not available!')
            return [], False

        self.get_logger().info(f'📍 Requesting path from ({start_pose.pose.position.x:.3f}, {start_pose.pose.position.y:.3f}) to ({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f})')

        request = ComputePath.Request()
        request.start_pose = start_pose
        request.goal_pose = goal_pose

        future = self.path_client.call_async(request)
        
        # Wait for future to complete with proper spinning
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if not future.done():
            self.get_logger().error('❌ Path service call timed out after 30s')
            return [], False

        if not future.result():
            self.get_logger().error('❌ Path service call failed - no response received')
            return [], False

        response = future.result()

        if response.success:
            self.get_logger().info(f'✅ Path found with {len(response.stm32_commands)} commands')
            return response.stm32_commands, True
        else:
            self.get_logger().warning(f'⚠ Path failed: {response.message}')
            return [], False

    # =========================================================
    # 2️⃣ UPDATE MAP
    # =========================================================
    def update_map(self):
        """
        Trigger map update to fetch latest stable batches from ArUco mapper
        
        Returns:
            success (bool): Whether the update succeeded
            message (str): Status message
        """
        # Check if service is available
        if not self.update_map_client.service_is_ready():
            self.get_logger().error('❌ /update_map service is not available!')
            return False, 'Service not available'

        self.get_logger().info('🔄 Requesting map update...')

        request = Trigger.Request()
        future = self.update_map_client.call_async(request)
        
        # Wait for future to complete with proper spinning
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error('❌ Map update service call timed out after 10s')
            return False, 'Timeout'

        if not future.result():
            self.get_logger().error('❌ Map update service call failed - no response received')
            return False, 'No response'

        response = future.result()

        if response.success:
            self.get_logger().info(f'✅ Map updated: {response.message}')
            return True, response.message
        else:
            self.get_logger().warning(f'⚠️ Map update failed: {response.message}')
            return False, response.message

    # =========================================================
    # RESET ARUCO STABILITY
    # =========================================================
    def reset_stability(self):
        """
        Trigger /reset_map on pathfinder map manager
        
        Returns:
            success (bool): Whether the reset succeeded
            message (str): Status message
        """
        # Check if service is available
        if not self.reset_stability_client.service_is_ready():
            self.get_logger().error('❌ /reset_map service is not available!')
            return False, 'Service not available'

        self.get_logger().info('🔄 Calling /reset_map service...')

        request = Trigger.Request()
        future = self.reset_stability_client.call_async(request)
        
        # Wait for future to complete with proper spinning
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error('❌ Reset stability service call timed out after 10s')
            return False, 'Timeout'

        if not future.result():
            self.get_logger().error('❌ Reset stability service call failed - no response received')
            return False, 'No response'

        response = future.result()

        if response.success:
            self.get_logger().info(f'✅ Stability reset: {response.message}')
            return True, response.message
        else:
            self.get_logger().warning(f'⚠️ Stability reset failed: {response.message}')
            return False, response.message

    # =========================================================
    # CLEAR MAP
    # =========================================================
    def clear_map(self):
        """
        Clear all map obstacles and reset state for fresh update_map call
        
        Returns:
            success (bool): Whether the clear succeeded
            message (str): Status message
        """
        # Check if service is available
        if not self.clear_map_client.service_is_ready():
            self.get_logger().error('❌ /clear_map service is not available!')
            return False, 'Service not available'

        self.get_logger().info('🗑️ Requesting map clear...')

        request = Trigger.Request()
        future = self.clear_map_client.call_async(request)
        
        # Wait for future to complete with proper spinning
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error('❌ Clear map service call timed out after 10s')
            return False, 'Timeout'

        if not future.result():
            self.get_logger().error('❌ Clear map service call failed - no response received')
            return False, 'No response'

        response = future.result()

        if response.success:
            self.get_logger().info(f'✅ Map cleared: {response.message}')
            return True, response.message
        else:
            self.get_logger().warning(f'⚠️ Map clear failed: {response.message}')
            return False, response.message

    # =========================================================
    # 3️⃣ GET BATCH COLOR
    # =========================================================
    def get_batch_color(self, known_id: str, pos: Point):
        """
        Request color pattern for a batch ID

        Returns:
            color_pattern (str)
            success (bool)
            direction (str)
            batch_center (Point)
            known_id (str) - The actual batch known_id from the response
        """

        request = GetBatchColorPattern.Request()
        request.batch_known_id = known_id
        request.pos = pos

        future = self.batch_color_client.call_async(request)
        
        # Wait for future to complete with proper spinning
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error('❌ Batch color service call timed out after 10s')
            return "", False, "", Point(), ""

        if not future.result():
            self.get_logger().error('❌ Batch color service call failed')
            return "", False, "", Point(), ""

        response = future.result()

        if response.success:
            return response.color_pattern, True, response.direction, response.batch_center, response.message
        else:
            self.get_logger().warning(f'⚠ Batch error: {response.message}')
            return "", False, "", Point(), ""
    
    def delete_batch(self, known_id: str):
        """
        Delete a batch by its known_id (after it's been picked up)
        
        Args:
            known_id: The batch known_id to delete (e.g., 'f1', 's2', 'g3')
        
        Returns:
            success (bool): True if batch was deleted, False otherwise
        """
        if not known_id:
            self.get_logger().warning('⚠️  Empty known_id provided to delete_batch')
            return False
        
        request = DeleteBatch.Request()
        request.known_id = known_id
        
        future = self.delete_batch_client.call_async(request)
        
        # Wait for future to complete with proper spinning
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error(f'❌ Delete batch service call timed out after 5s')
            return False
        
        if not future.result():
            self.get_logger().error('❌ Delete batch service call failed')
            return False
        
        response = future.result()
        
        if response.success:
            self.get_logger().info(f'🗑️  Batch "{known_id}" deleted successfully')
            return True
        else:
            self.get_logger().warning(f'⚠️  Failed to delete batch "{known_id}": {response.message}')
            return False
    
    def get_active_batches(self):
        """
        Get information about all active batches
        
        Returns:
            Dictionary with batch information arrays or None on error:
            {
                'known_ids': List[str],
                'batch_types': List[str],  # 'fx', 'sx', 'gx'
                'member_counts': List[int],
                'directions': List[str],  # 'horizontal', 'vertical', or ''
                'centers': List[Point],
                'qualities': List[int]
            }
        """
        request = GetActiveBatches.Request()
        
        future = self.get_active_batches_client.call_async(request)
        
        # Wait for future to complete with proper spinning
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error(f'❌ Get active batches service call timed out after 5s')
            return None
        
        if not future.result():
            self.get_logger().error('❌ Get active batches service call failed')
            return None
        
        response = future.result()
        
        # Convert response to dictionary
        batch_info = {
            'known_ids': list(response.known_ids),
            'batch_types': list(response.batch_types),
            'member_counts': list(response.member_counts),
            'directions': list(response.directions),
            'centers': list(response.centers),
            'qualities': list(response.qualities)
        }
        
        self.get_logger().info(
            f'📊 Retrieved {len(batch_info["known_ids"])} active batches: '
            f'FX={batch_info["batch_types"].count("fx")}, '
            f'SX={batch_info["batch_types"].count("sx")}'
        )
        
        return batch_info


def main(args=None):
    rclpy.init(args=args)
    node = PathfinderClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()