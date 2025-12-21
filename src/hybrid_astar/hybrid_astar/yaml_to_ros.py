#!/usr/bin/env python3
"""
YAML → ArucoMapperState → ComputePath
Fully equivalent to JSON pipeline
"""

import yaml
import math
import time
import rclpy
from rclpy.node import Node

from hybrid_astar.msg import ArucoMapperState, Batch, BatchMember
from hybrid_astar.srv import ComputePath
from geometry_msgs.msg import Point, PoseStamped


class YAMLToPathPlanner(Node):

    def __init__(self, yaml_path: str):
        super().__init__('yaml_to_path_planner')

        self.yaml_path = yaml_path
        self.data = None

        self.pub = self.create_publisher(
            ArucoMapperState,
            '/aruco_mapper_state',
            10
        )

        self.client = self.create_client(
            ComputePath,
            '/compute_path'
        )

        self.get_logger().info("📦 YAML → Pathfinder node started")

        self.get_logger().info("⏳ Waiting for /compute_path service...")
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("❌ /compute_path service not available")

        self.get_logger().info("✅ /compute_path service available")

    # ---------------------------------------------------------
    def load_yaml(self):
        with open(self.yaml_path, 'r') as f:
            self.data = yaml.safe_load(f)

        if 'batches' not in self.data:
            raise ValueError("Missing 'batches'")
        if 'start_point' not in self.data:
            raise ValueError("Missing 'start_point'")
        if 'goal_point' not in self.data:
            raise ValueError("Missing 'goal_point'")

        self.get_logger().info(
            f"✅ Loaded YAML ({len(self.data['batches'])} batches)"
        )

    # ---------------------------------------------------------
    def publish_batches(self):
        msg = ArucoMapperState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.initialized = True
        msg.tracks = []
        msg.batches = []

        for b in self.data['batches']:
            batch = Batch()
            batch.batch_id = int(b['batch_id'])
            batch.known_id = str(b.get('known_id', ''))
            batch.direction = b['direction']
            batch.phi_deg = float(b.get('phi_deg', 0.0))
            batch.avg_adjacent_dist_m = float(
                b.get('avg_adjacent_dist_m', 0.0)
            )

            c = b['center']
            batch.center = Point(
                x=c['x'] / 1000.0,
                y=c['y'] / 1000.0,
                z=c.get('z', 0.0)
            )

            for m in b['members']:
                member = BatchMember()
                member.aruco_id = int(m['aruco_id'])
                member.color = m['color']

                p = m['map_pos']
                member.map_pos = Point(
                    x=p['x'] / 1000.0,
                    y=p['y'] / 1000.0,
                    z=p.get('z', 0.0)
                )
                batch.members.append(member)

            batch.count = len(batch.members)
            msg.batches.append(batch)

        self.pub.publish(msg)

        self.get_logger().info(
            f"🚀 Published {len(msg.batches)} batches "
            f"({sum(len(b.members) for b in msg.batches)} obstacles)"
        )

        # Important: let planner receive obstacles
        time.sleep(1.0)

    # ---------------------------------------------------------
    def request_path(self):
        s = self.data['start_point']
        g = self.data['goal_point']

        req = ComputePath.Request()

        # Start pose
        req.start_pose = PoseStamped()
        req.start_pose.header.frame_id = 'map'
        req.start_pose.header.stamp = self.get_clock().now().to_msg()
        req.start_pose.pose.position.x = s['x'] / 1000.0
        req.start_pose.pose.position.y = s['y'] / 1000.0

        theta = math.radians(s['theta'])
        req.start_pose.pose.orientation.z = math.sin(theta / 2.0)
        req.start_pose.pose.orientation.w = math.cos(theta / 2.0)

        # Goal pose
        req.goal_pose = PoseStamped()
        req.goal_pose.header.frame_id = 'map'
        req.goal_pose.header.stamp = self.get_clock().now().to_msg()
        req.goal_pose.pose.position.x = g['x'] / 1000.0
        req.goal_pose.pose.position.y = g['y'] / 1000.0

        theta = math.radians(g['theta'])
        req.goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        req.goal_pose.pose.orientation.w = math.cos(theta / 2.0)

        self.get_logger().info("🧭 Requesting path...")
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.result():
            self.get_logger().error("❌ Path planning failed")
            return

        res = future.result()
        if not res.success:
            self.get_logger().error(f"❌ {res.message}")
            return

        self.get_logger().info("✅ Path planned successfully")
        self.get_logger().info(f"   Waypoints: {res.num_waypoints}")
        self.get_logger().info(f"   Length: {res.path_length_m:.2f} m")
        self.get_logger().info(f"   Time: {res.planning_time_ms:.1f} ms")

        for cmd in res.stm32_commands:
            self.get_logger().info(f"   CMD → {cmd}")


# =============================================================
def main():
    import sys, os

    if len(sys.argv) < 2:
        print("Usage:")
        print("  ros2 run hybrid_astar yaml_to_path_planner.py map.yaml")
        sys.exit(1)

    yaml_file = os.path.abspath(sys.argv[1])
    if not os.path.exists(yaml_file):
        print(f"❌ File not found: {yaml_file}")
        sys.exit(1)

    rclpy.init()
    node = YAMLToPathPlanner(yaml_file)

    node.load_yaml()
    node.publish_batches()
    node.request_path()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()