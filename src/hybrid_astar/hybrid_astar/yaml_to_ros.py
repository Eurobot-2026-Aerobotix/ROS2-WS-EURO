#!/usr/bin/env python3
"""
YAML → ArucoMapperState → ComputePath
Fully equivalent to JSON pipeline
"""

import yaml
import json
import math
import time
import rclpy
from rclpy.node import Node

from hybrid_astar.msg import ArucoMapperState, Batch, BatchMember
from hybrid_astar.srv import ComputePath
from geometry_msgs.msg import Point, PoseStamped


class YAMLToPathPlanner(Node):

    def __init__(self, yaml_path: str, output_json: str = "result.json"):
        super().__init__('yaml_to_path_planner')

        self.yaml_path = yaml_path
        self.output_json = output_json
        self.data = None
        self.result_data = {
            "map_info": {
                "width": 2000.0,
                "height": 3000.0,
                "grid_resolution": 25.0,
                "window_width": 1200,
                "window_height": 800
            },
            "obstacles": [],
            "start_point": {},
            "goal_point": {},
            "path": []
        }

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

        # Store start and goal points
        s = self.data['start_point']
        self.result_data['start_point'] = {
            'x': float(s['x']),
            'y': float(s['y']),
            'theta': float(s['theta'])
        }

        g = self.data['goal_point']
        self.result_data['goal_point'] = {
            'x': float(g['x']),
            'y': float(g['y']),
            'theta': float(g['theta'])
        }

        # Convert batches to obstacles
        self._convert_batches_to_obstacles()

    # ---------------------------------------------------------
    def _convert_batches_to_obstacles(self):
        """Convert YAML batches to obstacle format"""
        for batch in self.data['batches']:
            direction = batch['direction']
            
            for member in batch['members']:
                pos = member['map_pos']
                
                # Determine obstacle dimensions based on direction
                if direction == 'vertical':
                    # Vertical orientation: narrow width, tall height
                    width = 50.0
                    height = 150.0
                elif direction == 'horizontal':
                    # Horizontal orientation: wide width, narrow height
                    width = 150.0
                    height = 50.0
                else:
                    # Default dimensions
                    width = 50.0
                    height = 150.0
                
                obstacle = {
                    'x': float(pos['x']),
                    'y': float(pos['y']),
                    'width': width,
                    'height': height,
                    'color': member['color']
                }
                self.result_data['obstacles'].append(obstacle)
        
        self.get_logger().info(
            f"✅ Converted {len(self.result_data['obstacles'])} obstacles"
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

        # Extract path from response
        self._extract_path_from_response(res)

    # ---------------------------------------------------------
    def _extract_path_from_response(self, response):
        """Extract path waypoints from the ComputePath response"""
        if hasattr(response, 'path') and response.path:
            # If response has a path attribute with poses
            for pose_stamped in response.path.poses:
                pose = pose_stamped.pose
                
                # Convert quaternion to theta
                z = pose.orientation.z
                w = pose.orientation.w
                theta_rad = 2.0 * math.atan2(z, w)
                theta_deg = math.degrees(theta_rad)
                
                waypoint = {
                    'x': pose.position.x * 1000.0,  # Convert m to mm
                    'y': pose.position.y * 1000.0,
                    'theta': theta_deg
                }
                self.result_data['path'].append(waypoint)
        
        self.get_logger().info(
            f"✅ Extracted {len(self.result_data['path'])} waypoints from path"
        )
        
        # Save result to JSON
        self._save_result_json()

    # ---------------------------------------------------------
    def _save_result_json(self):
        """Save result data to JSON file"""
        try:
            with open(self.output_json, 'w') as f:
                json.dump(self.result_data, f, indent=2)
            
            self.get_logger().info(f"✅ Saved result to {self.output_json}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save JSON: {e}")


# =============================================================
def main():
    import sys, os

    if len(sys.argv) < 2:
        print("Usage:")
        print("  ros2 run hybrid_astar yaml_to_path_planner.py map.yaml [output.json]")
        print("  Default output: result.json")
        sys.exit(1)

    yaml_file = os.path.abspath(sys.argv[1])
    if not os.path.exists(yaml_file):
        print(f"❌ File not found: {yaml_file}")
        sys.exit(1)

    # Optional output JSON file
    output_json = sys.argv[2] if len(sys.argv) > 2 else "result.json"

    rclpy.init()
    node = YAMLToPathPlanner(yaml_file, output_json)

    node.load_yaml()
    node.publish_batches()
    node.request_path()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()