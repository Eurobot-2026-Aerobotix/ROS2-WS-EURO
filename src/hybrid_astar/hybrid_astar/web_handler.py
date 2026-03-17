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
import requests
import socket
import uuid
from typing import List, Dict, Optional
import sys
import os

# ROS2 messages
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from aruco_mapper_msgs.msg import ArucoMapperState, Batch, BatchMember
from aruco_mapper_msgs.srv import GetStableBatches
from hybrid_astar.srv import ComputePath, GetBatchColorPattern, GetActiveBatches

# Import DecisionMaker from euro package
try:
    from euro.decision_maker import DecisionMaker
    from euro.front_systems_controller import FrontSystemsController
    from euro.back_systems_controller import BackSystemsController
    from euro.pneumatic_controller import PneumaticController
except ImportError:
    # Fallback: try adding to path if import fails (development mode)
    euro_pkg_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '../../../euro')
    )
    if os.path.exists(euro_pkg_root) and euro_pkg_root not in sys.path:
        sys.path.insert(0, euro_pkg_root)
    from euro.decision_maker import DecisionMaker
    from euro.front_systems_controller import FrontSystemsController
    from euro.back_systems_controller import BackSystemsController
    from euro.pneumatic_controller import PneumaticController


class RobotPose:
    """Simple robot pose container for DecisionMaker"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x  # meters
        self.y = y  # meters
        self.z = z  # radians or degrees


class PathfinderAdapter:
    """Adapter to make WebHandler compatible with DecisionMaker's pathfinder interface"""
    def __init__(self, web_handler, callback_group=None):
        self.web_handler = web_handler
        self.goal_angle = None
        self.untracked_batches = []
        
        # Create service clients with callback group for concurrent execution
        self.get_active_batches_client = web_handler.create_client(
            GetActiveBatches,
            '/pathfinder/get_active_batches',
            callback_group=callback_group
        )
        
        self.compute_path_client = web_handler.create_client(
            ComputePath,
            '/compute_path',
            callback_group=callback_group
        )

    @property
    def enemy_x(self):
        """Enemy x in meters (compatible with PathfinderClient interface)."""
        if not self.web_handler.enemy_pose:
            return None
        return self.web_handler.mm_to_m(self.web_handler.enemy_pose.get('x_mm', 0.0))

    @property
    def enemy_y(self):
        """Enemy y in meters (compatible with PathfinderClient interface)."""
        if not self.web_handler.enemy_pose:
            return None
        return self.web_handler.mm_to_m(self.web_handler.enemy_pose.get('y_mm', 0.0))
        
    def set_goal(self, x, y, theta):
        """Set goal pose (x, y in meters, theta in radians)"""
        self.web_handler.goal_pose = {
            'x_mm': x * 1000.0,
            'y_mm': y * 1000.0,
            'theta_deg': math.degrees(theta)
        }
        self.goal_angle = theta
        
    def path(self, start_x=None, start_y=None, start_theta=None):
        """Generate path - returns (stm32_commands, success)
        
        Args:
            start_x: Optional start x in meters (overrides web_handler.start_pose)
            start_y: Optional start y in meters (overrides web_handler.start_pose)
            start_theta: Optional start theta in radians (overrides web_handler.start_pose)
        """
        if not self.compute_path_client.service_is_ready():
            self.web_handler.get_logger().error('❌ /compute_path service not ready')
            return [], False

        # Build effective start pose: use provided values or fall back to web_handler.start_pose
        if start_x is not None and start_y is not None and start_theta is not None:
            effective_start = {
                'x_mm': start_x * 1000.0,
                'y_mm': start_y * 1000.0,
                'theta_deg': math.degrees(start_theta)
            }
        else:
            effective_start = self.web_handler.start_pose

        # Get start pose from web_handler
        if not effective_start:
            self.web_handler.get_logger().error('❌ Start pose not set')
            return [], False
        
        if not self.web_handler.goal_pose:
            self.web_handler.get_logger().error('❌ Goal pose not set')
            return [], False
        
        try:
            # Create service request
            request = ComputePath.Request()
            
            # Start pose
            request.start_pose = PoseStamped()
            request.start_pose.header.frame_id = 'map'
            request.start_pose.header.stamp = self.web_handler.get_clock().now().to_msg()
            request.start_pose.pose.position.x = self.web_handler.mm_to_m(effective_start['x_mm'])
            request.start_pose.pose.position.y = self.web_handler.mm_to_m(effective_start['y_mm'])
            request.start_pose.pose.position.z = 0.0
            request.start_pose.pose.orientation = self.web_handler.euler_to_quaternion(
                self.web_handler.deg_to_rad(effective_start['theta_deg'])
            )
            
            # Goal pose
            request.goal_pose = PoseStamped()
            request.goal_pose.header.frame_id = 'map'
            request.goal_pose.header.stamp = self.web_handler.get_clock().now().to_msg()
            request.goal_pose.pose.position.x = self.web_handler.mm_to_m(self.web_handler.goal_pose['x_mm'])
            request.goal_pose.pose.position.y = self.web_handler.mm_to_m(self.web_handler.goal_pose['y_mm'])
            request.goal_pose.pose.position.z = 0.0
            request.goal_pose.pose.orientation = self.web_handler.euler_to_quaternion(
                self.web_handler.deg_to_rad(self.web_handler.goal_pose['theta_deg'])
            )

            # Batch known_ids to ignore as obstacles during this planning request
            request.untracked_batches = list(self.untracked_batches)
            
            # Call service
            future = self.compute_path_client.call_async(request)
            
            # Use proper spinning
            import rclpy
            rclpy.spin_until_future_complete(self.web_handler, future, timeout_sec=10.0)
            
            if not future.done():
                self.web_handler.get_logger().error('❌ Path computation timeout')
                return [], False
            
            response = future.result()
            
            if not response.success:
                self.web_handler.get_logger().error(f'❌ Path computation failed: {response.message}')
                return [], False
            
            # Extract STM32 commands from response
            stm32_commands = list(response.stm32_commands)
            
            self.web_handler.get_logger().info(
                f'✅ Path computed: {response.num_waypoints} waypoints, {response.path_length_m:.2f}m'
            )
            
            return stm32_commands, True
            
        except Exception as e:
            self.web_handler.get_logger().error(f'❌ Error calling compute_path: {e}')
            import traceback
            self.web_handler.get_logger().error(traceback.format_exc())
            return [], False
    
    def get_active_batches(self):
        """
        Get information about all active batches via ROS2 service
        
        Returns:
            Dictionary with batch information arrays or None on error
        """
        if not self.get_active_batches_client.service_is_ready():
            self.web_handler.get_logger().warning('⚠️ /pathfinder/get_active_batches service not ready')
            return None
        
        request = GetActiveBatches.Request()
        
        try:
            # Call service and spin until complete (allows executor to process)
            future = self.get_active_batches_client.call_async(request)
            
            import rclpy
            rclpy.spin_until_future_complete(self.web_handler, future, timeout_sec=5.0)
            
            if not future.done():
                self.web_handler.get_logger().error('❌ Get active batches timeout')
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
            
            self.web_handler.get_logger().info(
                f'📊 Retrieved {len(batch_info["known_ids"])} active batches'
            )
            
            return batch_info
            
        except Exception as e:
            self.web_handler.get_logger().error(f'❌ Error calling get_active_batches: {e}')
            return None

    def get_batch_color(self, known_id: str, pos=None):
        """Get color pattern for a batch synchronously.
        Returns (color_pattern, success, direction, batch_center, known_id)
        """
        from hybrid_astar.srv import GetBatchColorPattern
        from geometry_msgs.msg import Point

        client = self.web_handler.batch_color_client
        if not client.service_is_ready():
            self.web_handler.get_logger().warning('⚠️ /get_batch_color_pattern service not ready')
            return '', False, '', Point(), ''

        request = GetBatchColorPattern.Request()
        request.batch_known_id = known_id
        request.pos = pos if pos is not None else Point()

        try:
            import rclpy
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.web_handler, future, timeout_sec=5.0)
            if not future.done():
                self.web_handler.get_logger().error('❌ get_batch_color timeout')
                return '', False, '', Point(), ''
            response = future.result()
            if response.success:
                return response.color_pattern, True, response.direction, response.batch_center, response.message
            else:
                return '', False, '', Point(), ''
        except Exception as e:
            self.web_handler.get_logger().error(f'❌ get_batch_color error: {e}')
            return '', False, '', Point(), ''

    def add_untracked_batch(self, known_id: str):
        """Add a batch known_id to be ignored as obstacle during path planning."""
        if not known_id:
            return
        if known_id not in self.untracked_batches:
            self.untracked_batches.append(known_id)
            self.web_handler.get_logger().info(f"🙈 Added untracked batch for planning: {known_id}")

    def remove_untracked_batch(self, known_id: str):
        """Remove a known_id from untracked planning list."""
        if known_id in self.untracked_batches:
            self.untracked_batches.remove(known_id)
            self.web_handler.get_logger().info(f"👁️ Restored tracked batch for planning: {known_id}")

    def clear_untracked_batches(self):
        """Clear all untracked batch known_ids used for planning."""
        if self.untracked_batches:
            self.web_handler.get_logger().info(f"🧹 Clearing untracked planning batches: {self.untracked_batches}")
        self.untracked_batches.clear()


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
        /web/steal_batch: Trigger batch stealing via DecisionMaker (JSON string)
    
    Publishes:
        /aruco_mapper/state_fx: Send batches to hybrid_astar_node
        /obstacle_relative: Send enemy obstacles to hybrid_astar_node (PoseStamped)
        /web/path_result: Return path to web (JSON string)
        /web/batch_color_result: Return batch color to web (JSON string)
        /web/steal_batch_result: Return steal batch result to web (JSON string)
    """
    
    def __init__(self):
        super().__init__('web_handler_node')

        # Runtime identity (helps detect duplicate web_handler instances)
        self.hostname = socket.gethostname()
        self.pid = os.getpid()
        self.instance_id = f"{self.hostname}:{self.pid}:{uuid.uuid4().hex[:8]}"
        
        # Create reentrant callback group for concurrent service calls
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.reentrant_group = ReentrantCallbackGroup()
        
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
        
        self.steal_batch_result_pub = self.create_publisher(
            String,
            '/web/steal_batch_result',
            10
        )

        self.test_controller_result_pub = self.create_publisher(
            String,
            '/web/test_controller_result',
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
        
        self.create_subscription(
            String,
            '/web/steal_batch',
            self.handle_steal_batch_request,
            10,
            callback_group=self.reentrant_group
        )

        self.create_subscription(
            String,
            '/web/test_controller',
            self.handle_test_controller_request,
            10,
            callback_group=self.reentrant_group
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

        self.update_map_client = self.create_client(
            Trigger,
            '/update_map',
            callback_group=self.reentrant_group
        )

        self.clear_map_client = self.create_client(
            Trigger,
            '/clear_map',
            callback_group=self.reentrant_group
        )

        self.reset_map_client = self.create_client(
            Trigger,
            '/reset_map',
            callback_group=self.reentrant_group
        )
        
        # Service servers
        self.get_stable_batches_server = self.create_service(
            GetStableBatches,
            '/aruco_mapper/get_stable_batches_web',
            self.handle_get_stable_batches_web
        )
        
        # Web server URL for fetching batches
        self.web_server_url = 'http://localhost:8000'
        
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

        # TEST page controller instances from euro package
        self.front_controller = FrontSystemsController('web_front_systems_controller')
        self.back_controller = BackSystemsController('web_back_systems_controller')
        self.pneumatic_controller = PneumaticController('web_pneumatic_controller')
        
        # Robot pose tracking for DecisionMaker
        self.latest_robot_pose = RobotPose()
        
        # Initialize DecisionMaker with reentrant callback group
        pathfinder_adapter = PathfinderAdapter(self, callback_group=self.reentrant_group)
        self.decision_maker = DecisionMaker(self, pathfinder_adapter)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🌐 Web Handler Node Ready!")
        self.get_logger().info(
            f"🆔 web_handler identity: host={self.hostname}, pid={self.pid}, instance_id={self.instance_id}"
        )
        self.get_logger().info("=" * 60)
        self.get_logger().info("📡 Listening for web dashboard requests via rosbridge...")
        self.get_logger().info("   Topics:")
        self.get_logger().info("   - /web/send_batches")
        self.get_logger().info("   - /web/start_pose")
        self.get_logger().info("   - /web/goal_pose")
        self.get_logger().info("   - /web/enemy_pose → /obstacle_relative")
        self.get_logger().info("   - /web/compute_path")
        self.get_logger().info("   - /web/steal_batch (DecisionMaker)")
        self.get_logger().info("   - /web/test_controller (manual euro controllers)")
        self.get_logger().info("   Services:")
        self.get_logger().info("   - /aruco_mapper/get_stable_batches_web (provides batches to map)")
        self.get_logger().info("   - /update_map, /clear_map, /reset_map (hybrid_astar controls)")
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
                batch.state = 'TRACKING'  # Set status to TRACKING for web-sent batches
                
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
            pose_msg.header.frame_id = 'map_web_enemy'
            
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
    
    def handle_steal_batch_request(self, msg: String):
        """
        Handle steal batch request from web
        
        Expected JSON: {
            "batch_type": "fx" or "sx" or null,
            "device": "ESP_front" or "ESP_back" or null,
            "min_members": 1
        }
        """
        try:
            data = json.loads(msg.data)
            batch_type = data.get('batch_type', None)
            device = data.get('device', None)
            min_members = data.get('min_members', 1)
            
            self.get_logger().info(
                f"🎯 Steal batch requested: type={batch_type or 'any'}, "
                f"device={device or 'auto'}, min_members={min_members}"
            )
            
            # Update robot pose from start pose if available
            if self.start_pose:
                self.latest_robot_pose.x = self.mm_to_m(self.start_pose['x_mm'])
                self.latest_robot_pose.y = self.mm_to_m(self.start_pose['y_mm'])
                self.latest_robot_pose.z = self.deg_to_rad(self.start_pose['theta_deg'])
            
            # Call DecisionMaker to steal batch
            tasks = self.decision_maker.steal_batch(
                batch_type=batch_type,
                device=device,
                min_members=min_members
            )
            
            if tasks:
                # steal_batch now returns a list of tasks [pickup_task, putdown_task]
                task_names = [task['name'] for task in tasks]
                self._log_steal_tasks_details(tasks)
                result = {
                    "success": True,
                    "message": f"Batch steal tasks generated: {', '.join(task_names)}",
                    "tasks": tasks,  # Array of tasks
                    "task_count": len(tasks)
                }
                self.get_logger().info(f"✅ Steal batch successful: {len(tasks)} tasks generated")
            else:
                result = {
                    "success": False,
                    "message": "No available batches to steal or path generation failed",
                    "tasks": None,
                    "task_count": 0
                }
                self.get_logger().warning("⚠️ Steal batch failed")
            
            # Publish result
            self.publish_steal_batch_result(result)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error handling steal batch: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.publish_steal_batch_result({
                "success": False,
                "message": f"Error: {str(e)}",
                "tasks": None,
                "task_count": 0
            })

    def handle_test_controller_request(self, msg: String):
        """
        Handle manual controller actions from the TEST dashboard page.

        Expected JSON examples:
            {"controller": "front", "action": "kabech", "value": 1}
            {"controller": "back", "action": "n20", "value": 1111}
            {"controller": "pneumatic", "side": "front", "action": "pump"}
        """
        data = None
        try:
            data = json.loads(msg.data)
            request_id = data.get('request_id')
            controller_name = str(data.get('controller', '')).strip().lower()
            action = str(data.get('action', '')).strip().lower()
            side = str(data.get('side', '')).strip().lower()
            value = data.get('value', None)

            if controller_name in ['front', 'back']:
                controller = self.front_controller if controller_name == 'front' else self.back_controller

                if action not in ['kabech', 'dawer', 'n20']:
                    raise ValueError(f"Unsupported action '{action}' for controller '{controller_name}'")

                if value is None:
                    raise ValueError(f"Missing value for {controller_name}.{action}")

                value_int = int(value)
                getattr(controller, action)(value_int)

                response = {
                    'success': True,
                    'controller': controller_name,
                    'action': action,
                    'value': value_int,
                    'request_id': request_id,
                    'message': f"Executed {controller_name}.{action}({value_int})"
                }
                self.get_logger().info(f"🧪 {response['message']}")

            elif controller_name == 'pneumatic':
                if side not in ['front', 'back']:
                    raise ValueError("Pneumatic command requires side='front' or 'back'")

                if action == 'pump':
                    command_value = 0
                elif action == 'disable_pump':
                    command_value = 15
                else:
                    raise ValueError(f"Unsupported pneumatic action '{action}'")

                if side == 'front':
                    self.pneumatic_controller.front(command_value)
                else:
                    self.pneumatic_controller.back(command_value)

                response = {
                    'success': True,
                    'controller': controller_name,
                    'action': action,
                    'side': side,
                    'value': command_value,
                    'request_id': request_id,
                    'message': f"Executed pneumatic.{side}({command_value})"
                }
                self.get_logger().info(f"🧪 {response['message']}")

            elif controller_name in ['hybrid_astar', 'pathfinder', 'map']:
                action_to_service = {
                    'update_map': ('/update_map', self.update_map_client),
                    'clear_map': ('/clear_map', self.clear_map_client),
                    'reset_map': ('/reset_map', self.reset_map_client),
                    # UI-friendly alias
                    'reset_beacon': ('/reset_map', self.reset_map_client),
                }

                if action not in action_to_service:
                    raise ValueError(
                        f"Unsupported action '{action}' for controller '{controller_name}'"
                    )

                service_name, client = action_to_service[action]
                success, service_message = self._call_trigger_service(client, service_name)

                response = {
                    'success': success,
                    'controller': 'hybrid_astar',
                    'action': action,
                    'request_id': request_id,
                    'message': (
                        f"Executed hybrid_astar.{action} via {service_name}: {service_message}"
                        if success
                        else f"Failed hybrid_astar.{action} via {service_name}: {service_message}"
                    )
                }

                if success:
                    self.get_logger().info(f"🧪 {response['message']}")
                else:
                    self.get_logger().warning(f"⚠️ {response['message']}")

            else:
                raise ValueError(f"Unknown controller '{controller_name}'")

            self.publish_test_controller_result(response)

        except Exception as e:
            self.get_logger().error(f"❌ Error handling TEST controller command: {e}")
            self.publish_test_controller_result({
                'success': False,
                'request_id': data.get('request_id') if isinstance(data, dict) else None,
                'message': f'Error: {str(e)}'
            })

    def _call_trigger_service(self, client, service_name: str, timeout_sec: float = 5.0):
        """Call a Trigger service and return (success, message)."""
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return False, f"{service_name} not available"

            request = Trigger.Request()
            future = client.call_async(request)

            import rclpy
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if not future.done():
                return False, f"{service_name} timeout"

            response = future.result()
            if response is None:
                return False, f"{service_name} returned no response"

            message = response.message or ("Service call succeeded" if response.success else "Service call failed")
            return bool(response.success), message
        except Exception as e:
            return False, f"{service_name} error: {str(e)}"

    def _log_steal_tasks_details(self, tasks: List[Dict]):
        """Log generated steal tasks in compact form for debugging."""
        self.get_logger().info(f"🧾 Generated {len(tasks)} steal task(s):")

        for idx, task in enumerate(tasks, 1):
            task_name = task.get('name', f'task_{idx}')
            sequence = task.get('sequence', [])
            global_goal = task.get('global_goal', {})

            self.get_logger().info(
                f"   [{idx}] {task_name} | steps={len(sequence)} | "
                f"goal=({global_goal.get('x', '?')}, {global_goal.get('y', '?')}, {global_goal.get('theta', '?')})"
            )

            # Only print command skeleton (no full JSON blob)
            for step_i, step in enumerate(sequence, 1):
                device = step.get('device', '?')
                command = step.get('command', '?')
                self.get_logger().info(f"      - step {step_i}: {device}.{command}")
    
    # ========================================================================
    # Service Calls
    # ========================================================================
    
    def handle_get_stable_batches_web(self, request: GetStableBatches.Request, response: GetStableBatches.Response) -> GetStableBatches.Response:
        """
        Service handler for /aruco_mapper/get_stable_batches_web
        Fetches batches from the web server (server.py) and returns them as ArucoMapperState
        """
        self.get_logger().info(f"🔍 GetStableBatches WEB service called (filter: '{request.batch_types_filter}')")
        
        try:
            # Fetch batches from web server
            web_response = requests.get(f"{self.web_server_url}/web_batches", timeout=5.0)
            
            if web_response.status_code != 200:
                self.get_logger().error(f"❌ Failed to fetch batches from web server: HTTP {web_response.status_code}")
                self.get_logger().error(f"   Response text: {web_response.text[:200]}")
                response.stable_state = ArucoMapperState()
                response.stable_state.stamp = self.get_clock().now().to_msg()
                response.stable_state.initialized = False
                return response
            
            web_data = web_response.json()
            web_batches = web_data.get('batches', [])
            
            self.get_logger().info(f"📦 Retrieved {len(web_batches)} batches from web server")
            
            # Convert web batches to ArucoMapperState
            aruco_state = ArucoMapperState()
            aruco_state.stamp = self.get_clock().now().to_msg()
            aruco_state.initialized = True
            
            for web_batch in web_batches:
                # Filter by known_id if specified
                if request.specific_known_id and web_batch.get('known_id') != request.specific_known_id:
                    continue
                
                # NOTE: Skip batch_types_filter for web batches since they're manually selected
                # Web batches can have any known_id format, not just 's', 'f', 'g' prefixes
                
                # Create Batch message
                batch = Batch()
                batch.batch_id = web_batch.get('batch_id', 0)
                batch.known_id = web_batch.get('known_id', '')
                batch.direction = web_batch.get('direction', 'horizontal')
                batch.count = web_batch.get('count', 4)
                batch.state = 'TRACKING'  # Set as TRACKING for web batches
                
                # Calculate pattern from members
                members_data = web_batch.get('members', [])
                pattern_chars = []
                for member in members_data:
                    color = member.get('color', 'yellow')
                    pattern_chars.append('y' if color == 'yellow' else 'b')
                batch.pattern = ''.join(pattern_chars)
                
                # Set batch center (average of member positions or from batch data)
                if members_data:
                    avg_x = sum(m.get('position', {}).get('x', 0) for m in members_data) / len(members_data)
                    avg_y = sum(m.get('position', {}).get('y', 0) for m in members_data) / len(members_data)
                    batch.center = Point()
                    batch.center.x = self.mm_to_m(avg_x)
                    batch.center.y = self.mm_to_m(avg_y)
                    batch.center.z = 0.0
                
                # Add members
                for member_data in members_data:
                    member = BatchMember()
                    member.aruco_id = member_data.get('aruco_id', 0)
                    member.color = member_data.get('color', 'yellow')
                    
                    # Position (convert from mm to m)
                    position = member_data.get('position', {})
                    member.map_pos = Point()
                    member.map_pos.x = self.mm_to_m(position.get('x', 0))
                    member.map_pos.y = self.mm_to_m(position.get('y', 0))
                    member.map_pos.z = 0.0
                    
                    batch.members.append(member)
                
                aruco_state.batches.append(batch)
            
            response.stable_state = aruco_state
            self.get_logger().info(f"✅ Returning {len(aruco_state.batches)} batches to caller")
            return response
            
        except requests.RequestException as e:
            self.get_logger().error(f"❌ Error fetching batches from web server: {e}")
            self.get_logger().error("   Is the Flask server running on port 8000?")
            response.stable_state = ArucoMapperState()
            response.stable_state.stamp = self.get_clock().now().to_msg()
            response.stable_state.initialized = False
            return response
        except Exception as e:
            self.get_logger().error(f"❌ Error processing batches: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            response.stable_state = ArucoMapperState()
            response.stable_state.stamp = self.get_clock().now().to_msg()
            response.stable_state.initialized = False
            return response
    
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

            # Default: track all batches unless explicitly ignored by DecisionMaker via PathfinderAdapter
            request.untracked_batches = []
            
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
    
    def publish_steal_batch_result(self, result: Dict):
        """Publish steal batch result to web dashboard"""
        msg = String()
        msg.data = json.dumps(result)
        self.steal_batch_result_pub.publish(msg)
        self.get_logger().info("📤 Steal batch result sent to web")

    def publish_test_controller_result(self, result: Dict):
        """Publish TEST controller command result to web dashboard"""
        if isinstance(result, dict):
            result.setdefault('handler_pid', self.pid)
            result.setdefault('handler_host', self.hostname)
            result.setdefault('handler_instance', self.instance_id)

        msg = String()
        msg.data = json.dumps(result)
        self.test_controller_result_pub.publish(msg)
        self.get_logger().info("📤 TEST controller result sent to web")


def main(args=None):
    rclpy.init(args=args)
    node = WebHandlerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.front_controller.destroy_node()
        node.back_controller.destroy_node()
        node.pneumatic_controller.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
