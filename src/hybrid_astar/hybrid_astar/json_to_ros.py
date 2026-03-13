#!/usr/bin/env python3
"""
JSON to ROS2 Path Planner Bridge
Loads obstacles from JSON, converts to batches, publishes as ArucoMapperState,
requests path via ROS2 service, saves result to JSON
"""

# Fix Python path to avoid namespace collision when script is symlinked
import sys
import os
script_dir = os.path.dirname(os.path.realpath(__file__))
if script_dir in sys.path:
    sys.path.remove(script_dir)

import json
import rclpy
from rclpy.node import Node
from aruco_mapper_msgs.msg import ArucoMapperState, Batch, BatchMember
from hybrid_astar.srv import ComputePath
from geometry_msgs.msg import PoseStamped, Point
from builtin_interfaces.msg import Time
import math
import time
import sys
import os


class JSONToROSBridge(Node):
    def __init__(self, json_file_path):
        super().__init__('json_to_ros_bridge')
        
        self.json_file_path = json_file_path
        self.data = None
        
        # Publishers
        self.aruco_state_pub = self.create_publisher(ArucoMapperState, '/aruco_mapper_state', 10)
        
        # Service client
        self.path_client = self.create_client(ComputePath, '/compute_path')
        
        # Wait for service
        self.get_logger().info('Waiting for path planning service...')
        if not self.path_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('❌ Path planning service not available!')
            raise RuntimeError('Service timeout')
        self.get_logger().info('✅ Service available!')
        
    def load_json(self):
        """Load JSON file"""
        try:
            with open(self.json_file_path, 'r') as f:
                self.data = json.load(f)
            self.get_logger().info(f'✅ Loaded JSON: {self.json_file_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load JSON: {e}')
            return False
    
    def publish_obstacles(self):
        """Convert JSON obstacles to batches and publish as ArucoMapperState"""
        if not self.data or 'obstacles' not in self.data:
            self.get_logger().error('❌ No obstacles in JSON')
            return False
        
        obstacles_data = self.data['obstacles']
        
        if not isinstance(obstacles_data, list):
            self.get_logger().error('❌ Obstacles must be a list')
            return False
        
        self.get_logger().info(f'📦 Converting {len(obstacles_data)} obstacles to batches (4 per batch)...')
        
        # Group obstacles by orientation (horizontal vs vertical)
        horizontal_obs = []
        vertical_obs = []
        
        for i, obs_data in enumerate(obstacles_data):
            width = float(obs_data.get('width', 50.0))
            height = float(obs_data.get('height', 150.0))
            
            obs_info = {
                'index': i,
                'aruco_id': i + 1,  # Simulated ArUco ID
                'x': float(obs_data.get('x', 0.0)),
                'y': float(obs_data.get('y', 0.0)),
                'width': width,
                'height': height,
                'color': obs_data.get('color', 'yellow')
            }
            
            # Determine orientation based on dimensions
            if width > height:
                horizontal_obs.append(obs_info)
            else:
                vertical_obs.append(obs_info)
        
        # Create batches (4 obstacles per batch)
        batches = []
        batch_id = 0
        known_id= 0
        # Create vertical batches - group by proximity
        # Vertical obstacles should be grouped by Y position first (same row), then by X
        if vertical_obs:
            # Sort by Y first, then X - groups obstacles in same row together
            vertical_obs.sort(key=lambda o: (round(o['y'] / 1000), o['x']))
            
            for i in range(0, len(vertical_obs), 4):
                batch_obstacles = vertical_obs[i:i+4]
                batches.append(self.create_batch(batch_id, batch_obstacles, 'vertical',str(batch_id)))
                batch_id += 1
        
        # Create horizontal batches - group by proximity
        # Horizontal obstacles should be grouped by X position first (same column), then by Y
        if horizontal_obs:
            # Sort by X first, then Y - groups obstacles in same column together
            horizontal_obs.sort(key=lambda o: (round(o['x'] / 1000), o['y']))
            
            for i in range(0, len(horizontal_obs), 4):
                batch_obstacles = horizontal_obs[i:i+4]
                batches.append(self.create_batch(batch_id, batch_obstacles, 'horizontal',str(batch_id)))
                batch_id += 1
    
        
        # Create and publish ArucoMapperState message
        msg = ArucoMapperState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.initialized = True
        msg.tracks = []  # Empty for now
        msg.batches = batches
        
        self.aruco_state_pub.publish(msg)
        self.get_logger().info(f'✅ Published {len(batches)} batches with {len(obstacles_data)} total obstacles')
        
        # Log batch summary
        for i, batch in enumerate(batches):
            self.get_logger().info(f'   Batch {i}: {batch.direction}, {batch.count} obstacles')
        
        # Give time for obstacles to be processed
        time.sleep(1.0)
        return True
    
    def create_batch(self, batch_id, obstacles, direction,known_id):
        """Create a Batch message from a list of obstacles"""
        batch = Batch()
        batch.batch_id = batch_id
        batch.count = len(obstacles)
        batch.direction = direction
        batch.known_id = known_id
        batch.phi_deg = 0.0
        batch.avg_adjacent_dist_m = 0.0
        
        # Calculate center of batch (average of obstacle centers)
        if obstacles:
            avg_x = sum(o['x'] for o in obstacles) / len(obstacles)
            avg_y = sum(o['y'] for o in obstacles) / len(obstacles)
            batch.center = Point(x=avg_x / 1000.0, y=avg_y / 1000.0, z=0.0)
        
        # Create members
        for obs in obstacles:
            member = BatchMember()
            member.aruco_id = obs['aruco_id']
            member.color = obs['color']
            # IMPORTANT: JSON has CENTER positions, but map_pos should be CENTER too
            # The hybrid_astar_node will convert to top-left corner when needed
            member.map_pos = Point(
                x=obs['x'] / 1000.0,  # Center X (convert mm to m)
                y=obs['y'] / 1000.0,  # Center Y (convert mm to m)
                z=0.0
            )
            batch.members.append(member)
        
        return batch
    
    def request_path(self):
        """Request path from ROS2 service"""
        if not self.data:
            self.get_logger().error('❌ No JSON data loaded')
            return None
        
        if 'start_point' not in self.data or 'goal_point' not in self.data:
            self.get_logger().error('❌ Missing start_point or goal_point in JSON')
            return None
        
        start = self.data['start_point']
        goal = self.data['goal_point']
        
        self.get_logger().info('='*60)
        self.get_logger().info('🔍 Requesting path:')
        self.get_logger().info(f'   Start: ({start["x"]:.1f}, {start["y"]:.1f}) θ={start["theta"]:.1f}°')
        self.get_logger().info(f'   Goal:  ({goal["x"]:.1f}, {goal["y"]:.1f}) θ={goal["theta"]:.1f}°')
        
        request = ComputePath.Request()
        
        # Start pose (convert mm to m)
        request.start_pose = PoseStamped()
        request.start_pose.header.stamp = self.get_clock().now().to_msg()
        request.start_pose.header.frame_id = 'map'
        request.start_pose.pose.position.x = start['x'] / 1000.0
        request.start_pose.pose.position.y = start['y'] / 1000.0
        request.start_pose.pose.position.z = 0.0
        
        # Convert theta from degrees to quaternion
        # Negate angle: JSON uses left=positive, ROS2/math uses CCW from east=positive
        theta_rad = math.radians(start['theta'])
        request.start_pose.pose.orientation.z = math.sin(theta_rad / 2.0)
        request.start_pose.pose.orientation.w = math.cos(theta_rad / 2.0)
        
        # Goal pose (convert mm to m)
        request.goal_pose = PoseStamped()
        request.goal_pose.header.stamp = self.get_clock().now().to_msg()
        request.goal_pose.header.frame_id = 'map'
        request.goal_pose.pose.position.x = goal['x'] / 1000.0
        request.goal_pose.pose.position.y = goal['y'] / 1000.0
        request.goal_pose.pose.position.z = 0.0
        
        # Convert theta from degrees to quaternion
        # Negate angle: JSON uses left=positive, ROS2/math uses CCW from east=positive
        theta_rad = math.radians(goal['theta'])
        request.goal_pose.pose.orientation.z = math.sin(theta_rad / 2.0)
        request.goal_pose.pose.orientation.w = math.cos(theta_rad / 2.0)

        # No batch ignored by default
        request.untracked_batches = []

        # Call service
        future = self.path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ {response.message}')
                self.get_logger().info(f'   Waypoints: {response.num_waypoints}')
                self.get_logger().info(f'   Path length: {response.path_length_m:.2f}m')
                self.get_logger().info(f'   Planning time: {response.planning_time_ms:.1f}ms')
                self.get_logger().info(f'   Obstacles used: {response.num_obstacles_used}')
                self.get_logger().info(f'   STM32 Commands: {len(response.stm32_commands)} commands')
                for cmd in response.stm32_commands:
                    self.get_logger().info(f'      {cmd}')
                self.get_logger().info('='*60)
                return response.path
            else:
                self.get_logger().error(f'❌ {response.message}')
                self.get_logger().info('='*60)
                return None
        else:
            self.get_logger().error('❌ Service call failed or timed out!')
            self.get_logger().info('='*60)
            return None
    
    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle in radians"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def save_path_to_json(self, path_msg):
        """Save path to JSON file"""
        if not path_msg or not path_msg.poses:
            self.get_logger().error('❌ No path to save')
            return False
        
        try:
            # Convert ROS2 path to JSON format
            path_json = []
            for pose_stamped in path_msg.poses:
                x_mm = pose_stamped.pose.position.x * 1000.0  # m to mm
                y_mm = pose_stamped.pose.position.y * 1000.0  # m to mm
                theta_rad = self.quaternion_to_yaw(pose_stamped.pose.orientation)
                theta_deg = math.degrees(theta_rad)
                
                path_json.append({
                    "x": round(x_mm, 2),
                    "y": round(y_mm, 2),
                    "theta": round(theta_deg, 2)
                })
            
            # Update JSON data
            self.data['path'] = path_json
            
            # Save to file
            with open(self.json_file_path, 'w') as f:
                json.dump(self.data, f, indent=2)
            
            self.get_logger().info(f'✅ Saved path ({len(path_json)} waypoints) to: {self.json_file_path}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to save JSON: {e}')
            return False


def main():
    # Parse arguments
    if len(sys.argv) < 2:
        print("Usage: ros2 run hybrid_astar json_to_ros.py <json_file>")
        print("Example: ros2 run hybrid_astar json_to_ros.py map.json")
        sys.exit(1)
    
    json_file = sys.argv[1]
    
    # Convert relative path to absolute
    if not json_file.startswith('/'):
        json_file = os.path.abspath(json_file)
    
    if not os.path.exists(json_file):
        print(f"❌ File not found: {json_file}")
        sys.exit(1)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        print("\n" + "="*60)
        print("   JSON to ROS2 Path Planner Bridge")
        print("="*60)
        print(f"📄 JSON file: {json_file}")
        print("="*60 + "\n")
        
        # Create bridge node
        bridge = JSONToROSBridge(json_file)
        
        # Step 1: Load JSON
        print("[Step 1/4] Loading JSON...")
        if not bridge.load_json():
            sys.exit(1)
        
        # Step 2: Publish obstacles as batches
        print("\n[Step 2/4] Converting obstacles to batches and publishing to ROS2...")
        if not bridge.publish_obstacles():
            sys.exit(1)
        
        # Step 3: Request path
        print("\n[Step 3/4] Requesting path from planner...")
        path = bridge.request_path()
        if path is None:
            print("\n❌ Path planning failed!")
            sys.exit(1)
        
        # Step 4: Save path to JSON
        print("\n[Step 4/4] Saving path to JSON...")
        if not bridge.save_path_to_json(path):
            sys.exit(1)
        
        print("\n" + "="*60)
        print("✅ SUCCESS! Path saved to JSON file")
        print("="*60 + "\n")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
