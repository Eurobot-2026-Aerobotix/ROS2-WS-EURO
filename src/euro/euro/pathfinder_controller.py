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

        self.get_logger().info('Waiting for services...')

        self.path_client.wait_for_service()
        self.batch_color_client.wait_for_service()

        self.get_logger().info('✅ Pathfinder services available')

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
            self.get_logger().info(
                f'✅ Path planned in {response.planning_time_ms:.2f} ms '
                f'with {response.num_waypoints} waypoints'
            )
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
        """

        request = GetBatchColorPattern.Request()
        request.known_id = known_id
        request.pos = pos

        future = self.batch_color_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error('❌ Batch color service call failed')
            return "", False

        response = future.result()

        if response.success:
            self.get_logger().info(
                f'🎨 Batch {known_id} → {response.color_pattern} '
                f'({response.direction})'
            )
            return response.color_pattern, True
        else:
            self.get_logger().warning(f'⚠ Batch error: {response.message}')
            return "", False


def main(args=None):
    rclpy.init(args=args)
    node = PathfinderClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Pathfinder client shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()