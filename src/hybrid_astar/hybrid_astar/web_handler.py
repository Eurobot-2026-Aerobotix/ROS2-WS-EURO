#!/usr/bin/env python3
"""
Web Handler Node - Bridge between Web Dashboard and ROS2
Connects web dashboard (via rosbridge WebSocket) to hybrid_astar_node

This node:
1. Subscribes to web commands (via rosbridge topics)
2. Converts units (mm→m, degrees→radians, euler→quaternion)
3. Publishes batches to /aruco_mapper_state
4. Calls /compute_path service
5. Returns results to web dashboard
"""

# Fix Python path to avoid namespace collision when script is symlinked
import sys
import os
script_dir = os.path.dirname(os.path.realpath(__file__))
if script_dir in sys.path:
    sys.path.remove(script_dir)

import rclpy
from rclpy.node import Node
import math
import json
from typing import List, Dict, Optional

# ROS2 messages
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from aruco_mapper_msgs.msg import ArucoMapperState, Batch, BatchMember
from hybrid_astar.srv import ComputePath, GetBatchColorPattern


class WebHandlerNode(Node):
    """
    Bridge between web dashboard and ROS2 pathfinding system
    
    Subscribes to:
        /web/send_batches: Receive batches from web (JSON string)
        /web/start_pose: Receive start pose from web (JSON string)
        /web/goal_pose: Receive goal pose from web (JSON string)
        /web/enemy_pose: Receive enemy pose from web (JSON string)
        /web/compute_path: Trigger path computation (JSON string)
        /web/get_batch_color: Query batch color pattern (JSON string)
    
    Publishes:
        /aruco_mapper/state_fx: Send batches to hybrid_astar_node
        /obstacle_relative: Send enemy obstacles to hybrid_astar_node (PoseStamped)
        /web/path_result: Return path to web (JSON string)
        /web/batch_color_result: Return batch color to web (JSON string)
    """
    
    def __init__(self):
        super().__init__('web_handler_node')
        
        # Publishers
        self.aruco_state_pub = self.create_publisher(
            ArucoMapperState,
            '/aruco_mapper/state_fx',
            10
        )
        
        self.path_result_pub = self.create_publisher(
            String,
            '/web/path_result',
            10
        )
        
        self.batch_color_result_pub = self.create_publisher(
            String,
            '/web/batch_color_result',
            10
        )
        
        self.obstacle_relative_pub = self.create_publisher(
            PoseStamped,
            '/obstacle_relative',
            10
        )
        
        # Subscribers
        self.create_subscription(
            String,
            '/web/send_batches',
            self.handle_send_batches,
            10
        )
        
        self.create_subscription(
            String,
            '/web/start_pose',
            self.handle_start_pose,
            10
        )
        
        self.create_subscription(
            String,
            '/web/goal_pose',
            self.handle_goal_pose,
            10
        )
        
        self.create_subscription(
            String,
            '/web/enemy_pose',
            self.handle_enemy_pose,
            10
        )
        
        self.create_subscription(
            String,
            '/web/compute_path',
            self.handle_compute_path_request,
            10
        )
        
        self.create_subscription(
            String,
            '/web/get_batch_color',
            self.handle_get_batch_color_request,
            10
        )
        
        # Service clients
        self.path_service_client = self.create_client(
            ComputePath,
            '/compute_path'
        )
        
        self.batch_color_client = self.create_client(
            GetBatchColorPattern,
            '/get_batch_color_pattern'
        )
        
        # Wait for services
        self.get_logger().info("⏳ Waiting for /compute_path service...")
        if not self.path_service_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("⚠️  /compute_path service not available yet")
        else:
            self.get_logger().info("✅ /compute_path service ready")
        
        self.get_logger().info("⏳ Waiting for /get_batch_color_pattern service...")
        if not self.batch_color_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("⚠️  /get_batch_color_pattern service not available yet")
        else:
            self.get_logger().info("✅ /get_batch_color_pattern service ready")
        
        # Storage for poses
        self.start_pose: Optional[Dict] = None
        self.goal_pose: Optional[Dict] = None
        self.enemy_pose: Optional[Dict] = None
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🌐 Web Handler Node Ready!")
        self.get_logger().info("=" * 60)
        self.get_logger().info("📡 Listening for web dashboard requests via rosbridge...")
        self.get_logger().info("   Topics:")
        self.get_logger().info("   - /web/send_batches")
        self.get_logger().info("   - /web/start_pose")
        self.get_logger().info("   - /web/goal_pose")
        self.get_logger().info("   - /web/enemy_pose → /obstacle_relative")
        self.get_logger().info("   - /web/compute_path")
        self.get_logger().info("=" * 60)
    
    # ========================================================================
    # Unit Conversion Utilities
    # ========================================================================
    
    def mm_to_m(self, value_mm: float) -> float:
        """Convert millimeters to meters"""
        return value_mm / 1000.0
    
    def m_to_mm(self, value_m: float) -> float:
        """Convert meters to millimeters"""
        return value_m * 1000.0
    
    def deg_to_rad(self, degrees: float) -> float:
        """Convert degrees to radians"""
        return math.radians(degrees)
    
    def rad_to_deg(self, radians: float) -> float:
        """Convert radians to degrees"""
        return math.degrees(radians)
    
    def euler_to_quaternion(self, yaw_rad: float) -> Quaternion:
        """
        Convert yaw angle (radians) to quaternion
        Web sends: theta in degrees (0° = down, 90° = right, CCW)
        ROS2 expects: quaternion
        """
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw_rad / 2.0)
        quat.w = math.cos(yaw_rad / 2.0)
        return quat
    
    def quaternion_to_yaw(self, quat: Quaternion) -> float:
        """
        Convert quaternion to yaw angle (radians)
        ROS2 returns: quaternion
        Web expects: theta in degrees
        """
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    # ========================================================================
    # Message Handlers
    # ========================================================================
    
    def handle_send_batches(self, msg: String):
        """
        Handle batches from web dashboard
        
        Expected JSON format:
        {
            "batches": [
                {
                    "batch_id": 1,
                    "known_id": "V_TOP_LEFT",
                    "direction": "horizontal",
                    "count": 4,
                    "members": [
                        {
                            "aruco_id": 47,
                            "position": {"x": 725.0, "y": 175.0},
                            "color": "yellow"
                        }
                    ]
                }
            ]
        }
        """
        try:
            data = json.loads(msg.data)
            web_batches = data.get('batches', [])
            
            if not web_batches:
                self.get_logger().warn("⚠️  No batches provided")
                return
            
            # Convert to ROS2 message
            aruco_msg = ArucoMapperState()
            aruco_msg.stamp = self.get_clock().now().to_msg()
            
            for web_batch in web_batches:
                batch = Batch()
                batch.batch_id = web_batch['batch_id']
                batch.known_id = web_batch['known_id']
                batch.direction = web_batch['direction']
                batch.count = web_batch['count']
                
                for web_member in web_batch['members']:
                    member = BatchMember()
                    member.aruco_id = web_member['aruco_id']
                    
                    # Convert position from mm to m
                    member.map_pos = Point()
                    member.map_pos.x = self.mm_to_m(web_member['position']['x'])
                    member.map_pos.y = self.mm_to_m(web_member['position']['y'])
                    member.map_pos.z = 0.0
                    
                    member.color = web_member['color']
                    batch.members.append(member)
                
                aruco_msg.batches.append(batch)
            
            # Publish to /aruco_mapper_state
            self.aruco_state_pub.publish(aruco_msg)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error handling batches: {e}")
    
    def handle_start_pose(self, msg: String):
        """
        Handle start pose from web
        
        Expected JSON: {"x": 200.0, "y": 400.0, "theta": -90.0}
        """
        try:
            data = json.loads(msg.data)
            self.start_pose = {
                'x_mm': data['x'],
                'y_mm': data['y'],
                'theta_deg': data['theta']
            }
            self.get_logger().info(
                f"🎯 Start pose: ({self.start_pose['x_mm']:.0f}, "
                f"{self.start_pose['y_mm']:.0f}) mm, "
                f"θ={self.start_pose['theta_deg']:.1f}°"
            )
        except Exception as e:
            self.get_logger().error(f"❌ Error parsing start pose: {e}")
    
    def handle_goal_pose(self, msg: String):
        """
        Handle goal pose from web
        
        Expected JSON: {"x": 2800.0, "y": 1600.0, "theta": 180.0}
        """
        try:
            data = json.loads(msg.data)
            self.goal_pose = {
                'x_mm': data['x'],
                'y_mm': data['y'],
                'theta_deg': data['theta']
            }
            self.get_logger().info(
                f"🏁 Goal pose: ({self.goal_pose['x_mm']:.0f}, "
                f"{self.goal_pose['y_mm']:.0f}) mm, "
                f"θ={self.goal_pose['theta_deg']:.1f}°"
            )
        except Exception as e:
            self.get_logger().error(f"❌ Error parsing goal pose: {e}")
    
    def handle_enemy_pose(self, msg: String):
        """
        Handle enemy pose from web and publish as obstacle
        
        Expected JSON: {"x": 1500.0, "y": 1000.0, "theta": 0.0}
        """
        try:
            data = json.loads(msg.data)
            self.enemy_pose = {
                'x_mm': data['x'],
                'y_mm': data['y'],
                'theta_deg': data['theta']
            }
            self.get_logger().info(
                f"⚔️  Enemy pose: ({self.enemy_pose['x_mm']:.0f}, "
                f"{self.enemy_pose['y_mm']:.0f}) mm, "
                f"θ={self.enemy_pose['theta_deg']:.1f}°"
            )
            
            # Publish to /obstacle_relative as PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            # Convert from mm to m
            pose_msg.pose.position.x = self.mm_to_m(self.enemy_pose['x_mm'])
            pose_msg.pose.position.y = self.mm_to_m(self.enemy_pose['y_mm'])
            pose_msg.pose.position.z = 0.0
            
            # Convert from degrees to radians and create quaternion
            theta_rad = self.deg_to_rad(self.enemy_pose['theta_deg'])
            pose_msg.pose.orientation = self.euler_to_quaternion(theta_rad)
            
            self.obstacle_relative_pub.publish(pose_msg)
            self.get_logger().info(
                f"📤 Published enemy obstacle to /obstacle_relative"
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ Error parsing enemy pose: {e}")
    
    def handle_compute_path_request(self, msg: String):
        """
        Handle path computation request from web
        
        Expected JSON: {} (empty, uses stored start/goal poses)
        """
        self.get_logger().info("🔍 Path computation requested from web")
        
        if self.start_pose is None or self.goal_pose is None:
            self.get_logger().error("❌ Start or goal pose not set!")
            self.publish_path_result({
                "success": False,
                "message": "Start or goal pose not set",
                "path": []
            })
            return
        
        # Call compute_path asynchronously
        self.compute_path_async()
    
    def handle_get_batch_color_request(self, msg: String):
        """
        Handle batch color pattern query
        
        Expected JSON: {"known_id": "V_TOP_LEFT"}
        """
        try:
            data = json.loads(msg.data)
            known_id = data.get('known_id', '')
            
            if not known_id:
                self.get_logger().warn("⚠️  No known_id provided")
                return
            
            self.get_logger().info(f"🔍 Querying batch color for '{known_id}'")
            self.get_batch_color_async(known_id)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error parsing batch color request: {e}")
    
    # ========================================================================
    # Service Calls
    # ========================================================================
    
    def compute_path_async(self):
        """Call /compute_path service asynchronously"""
        if not self.path_service_client.service_is_ready():
            self.get_logger().error("❌ /compute_path service not ready!")
            self.publish_path_result({
                "success": False,
                "message": "Path service not ready",
                "path": []
            })
            return
        
        try:
            # Create service request
            request = ComputePath.Request()
            
            # Start pose (convert mm → m, deg → rad → quaternion)
            request.start_pose = PoseStamped()
            request.start_pose.header.frame_id = 'map'
            request.start_pose.header.stamp = self.get_clock().now().to_msg()
            request.start_pose.pose.position.x = self.mm_to_m(self.start_pose['x_mm'])
            request.start_pose.pose.position.y = self.mm_to_m(self.start_pose['y_mm'])
            request.start_pose.pose.position.z = 0.0
            request.start_pose.pose.orientation = self.euler_to_quaternion(
                self.deg_to_rad(self.start_pose['theta_deg'])
            )
            
            # Goal pose (convert mm → m, deg → rad → quaternion)
            request.goal_pose = PoseStamped()
            request.goal_pose.header.frame_id = 'map'
            request.goal_pose.header.stamp = self.get_clock().now().to_msg()
            request.goal_pose.pose.position.x = self.mm_to_m(self.goal_pose['x_mm'])
            request.goal_pose.pose.position.y = self.mm_to_m(self.goal_pose['y_mm'])
            request.goal_pose.pose.position.z = 0.0
            request.goal_pose.pose.orientation = self.euler_to_quaternion(
                self.deg_to_rad(self.goal_pose['theta_deg'])
            )
            
            # Call service
            self.get_logger().info("📞 Calling /compute_path service...")
            future = self.path_service_client.call_async(request)
            future.add_done_callback(self.compute_path_callback)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error calling path service: {e}")
            self.publish_path_result({
                "success": False,
                "message": f"Error: {str(e)}",
                "path": []
            })
    
    def compute_path_callback(self, future):
        """Callback when /compute_path service returns"""
        try:
            response = future.result()
            
            if response.success:
                # Convert path from ROS2 format (m, quaternion) to web format (mm, degrees)
                web_path = []
                for pose_stamped in response.path.poses:
                    x_mm = self.m_to_mm(pose_stamped.pose.position.x)
                    y_mm = self.m_to_mm(pose_stamped.pose.position.y)
                    theta_rad = self.quaternion_to_yaw(pose_stamped.pose.orientation)
                    theta_deg = self.rad_to_deg(theta_rad)
                    
                    web_path.append({
                        'x': round(x_mm, 1),
                        'y': round(y_mm, 1),
                        'theta': round(theta_deg, 1)
                    })
                
                self.get_logger().info(
                    f"✅ Path computed: {response.num_waypoints} waypoints, "
                    f"{response.path_length_m:.2f}m, "
                    f"{response.planning_time_ms:.1f}ms"
                )
                
                result = {
                    "success": True,
                    "path": web_path,
                    "num_waypoints": response.num_waypoints,
                    "path_length_m": response.path_length_m,
                    "planning_time_ms": response.planning_time_ms,
                    "num_obstacles_used": response.num_obstacles_used,
                    "stm32_commands": list(response.stm32_commands),
                    "message": response.message
                }
                
                self.publish_path_result(result)
            else:
                self.get_logger().error(f"❌ Path planning failed: {response.message}")
                self.publish_path_result({
                    "success": False,
                    "message": response.message,
                    "path": []
                })
        
        except Exception as e:
            self.get_logger().error(f"❌ Error in path callback: {e}")
            self.publish_path_result({
                "success": False,
                "message": f"Callback error: {str(e)}",
                "path": []
            })
    
    def get_batch_color_async(self, known_id: str):
        """Call /get_batch_color_pattern service asynchronously"""
        if not self.batch_color_client.service_is_ready():
            self.get_logger().error("❌ /get_batch_color_pattern service not ready!")
            return
        
        try:
            request = GetBatchColorPattern.Request()
            request.batch_known_id = known_id
            
            future = self.batch_color_client.call_async(request)
            future.add_done_callback(
                lambda f: self.get_batch_color_callback(f, known_id)
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ Error calling batch color service: {e}")
    
    def get_batch_color_callback(self, future, known_id: str):
        """Callback when /get_batch_color_pattern service returns"""
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info(
                    f"✅ Batch '{known_id}': {response.color_pattern} "
                    f"({response.direction})"
                )
            else:
                self.get_logger().warn(f"❌ {response.message}")
            
            result = {
                "success": response.success,
                "known_id": known_id,
                "color_pattern": response.color_pattern,
                "direction": response.direction,
                "message": response.message
            }
            
            self.publish_batch_color_result(result)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error in batch color callback: {e}")
    
    # ========================================================================
    # Result Publishers
    # ========================================================================
    
    def publish_path_result(self, result: Dict):
        """Publish path result to web dashboard"""
        msg = String()
        msg.data = json.dumps(result)
        self.path_result_pub.publish(msg)
        self.get_logger().info("📤 Path result sent to web")
    
    def publish_batch_color_result(self, result: Dict):
        """Publish batch color result to web dashboard"""
        msg = String()
        msg.data = json.dumps(result)
        self.batch_color_result_pub.publish(msg)
        self.get_logger().info("📤 Batch color result sent to web")


def main(args=None):
    rclpy.init(args=args)
    node = WebHandlerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
