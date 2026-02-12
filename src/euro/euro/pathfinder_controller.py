#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path

from hybrid_astar.srv import ComputePath, GetBatchColorPattern

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

        self.path_client.wait_for_service()
        self.batch_color_client.wait_for_service()

        # Goal storage
        self.goal_x = None
        self.goal_y = None
        self.goal_angle = None

        # Enemy/obstacle position storage
        self.enemy_x = None
        self.enemy_y = None

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

    def enemy_callback(self, msg):
        """Update enemy obstacle position from /obstacle_relative topic"""
        self.enemy_x = msg.pose.position.x
        self.enemy_y = msg.pose.position.y

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

        request = ComputePath.Request()
        request.start_pose = start_pose
        request.goal_pose = goal_pose

        future = self.path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error('❌ Path service call failed')
            return [], False

        response = future.result()

        if response.success:
            return response.stm32_commands, True
        else:
            self.get_logger().warning(f'⚠ Path failed: {response.message}')
            return [], False

    # =========================================================
    # 2️⃣ GET BATCH COLOR
    # =========================================================
    def get_batch_color(self, known_id: str, pos: Point):
        """
        Request color pattern for a batch ID

        Returns:
            color_pattern (str)
            success (bool)
            direction (str)
            batch_center (Point)
        """

        request = GetBatchColorPattern.Request()
        request.batch_known_id = known_id
        request.pos = pos

        future = self.batch_color_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error('❌ Batch color service call failed')
            return "", False, "", Point()

        response = future.result()

        if response.success:
            return response.color_pattern, True, response.direction, response.batch_center
        else:
            self.get_logger().warning(f'⚠ Batch error: {response.message}')
            return "", False, "", Point()


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