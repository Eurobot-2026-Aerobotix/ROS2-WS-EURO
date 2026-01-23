#!/usr/bin/env python3
"""
Hybrid A* Path Planning Node for ROS2
Dynamic obstacle management with color-based filtering

This node uses the EXACT SAME advanced pathfinding logic as test_local_pathfinding.py,
including:
- Advanced smoothing and rotation validation
- Waypoint optimization (via HybridAStarPathfinder.smooth_path)
- Exact robot footprint collision detection
- The same motion primitives and cost functions

The only difference: this node gets obstacles from ROS2 topics and start/goal from
service requests, while the local script reads from JSON files.
"""

# Fix Python path to avoid namespace collision when script is symlinked
import sys
import os
script_dir = os.path.dirname(os.path.realpath(__file__))
if script_dir in sys.path:
    sys.path.remove(script_dir)

import rclpy
from rclpy.node import Node
from threading import Lock
import time
import math
from typing import Dict, List, Tuple, Optional

# ROS2 messages
from std_msgs.msg import Empty, Header
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

# Custom messages (will be generated)
from hybrid_astar.msg import Obstacle, ObstacleArray, ColorCommand
from aruco_mapper_msgs.msg import ArucoMapperState, Batch, BatchMember
from hybrid_astar.srv import ComputePath, GetBatchColorPattern

# Import the pathfinding algorithm
from hybrid_astar.hybrid_astar import HybridAStarPathfinder


class HybridAStarNode(Node):
    """
    ROS2 node for Hybrid A* pathfinding with dynamic obstacle management.
    
    Subscribes to:
        - /aruco_mapper/state_fx: Receive batches of obstacles from ArUco mapper (FX side)
        - /aruco_mapper/state_sx: Receive batches of obstacles from ArUco mapper (SX side)
        - /obstacles/clear_all: Clear all obstacles
        - /obstacle_relative: Receive enemy robot poses as obstacles (PoseStamped)
    
    Publishes:
        - /obstacles/current: Current obstacle list
        - /path_visualization: Path visualization for RViz
    
    Services:
        - /compute_path: Compute path from start to goal
    """
    
    def __init__(self):
        super().__init__('hybrid_astar_node')
        
        # Load all parameters
        self.load_parameters()
        
        # Predefined batch positions (in meters) for SX batches
        self.predefined_batch_positions = {
            "s1": {"x": 0.80, "y": 0.175, "phi_degrees": 0.0},
            "s2": {"x": 1.60, "y": 0.175, "phi_degrees": 0.0},
            "s3": {"x": 1.20, "y": 1.15, "phi_degrees": 90.0},
            "s4": {"x": 1.825, "y": 1.1, "phi_degrees": 90.0},
            "s5": {"x": 1.20, "y": 1.85, "phi_degrees": 90.0},
            "s6": {"x": 1.825, "y": 1.9, "phi_degrees": 90.0},
            "s7": {"x": 0.80, "y": 2.85, "phi_degrees": 90.0},
            "s8": {"x": 1.60, "y": 2.85, "phi_degrees": 90.0},
        }
        
        # Obstacle database (in-memory only) - now tracking source
        self.obstacles: List[Obstacle] = []
        self.obstacles_lock = Lock()
        
        # Track obstacle sources: 'fx', 'sx', or 'enemy'
        self.obstacle_sources: Dict[str, str] = {}  # obstacle_id -> source
        
        # Batch storage (for color pattern queries) - separate for fx and sx
        self.batches_fx: Dict[int, Batch] = {}
        self.batches_sx: Dict[int, Batch] = {}
        self.batches_lock = Lock()
        
        # Static obstacles - now using permanent area instead of JSON file
        self.static_obstacles: Dict[str, Obstacle] = {}
        # self.load_static_obstacles()  # DISABLED - using permanent area instead
        
        # Create subscribers
        self.create_subscription(
            ArucoMapperState, '/aruco_mapper/state_fx', self.handle_aruco_mapper_state_fx, 10)
        
        self.create_subscription(
            ArucoMapperState, '/aruco_mapper/state_sx', self.handle_aruco_mapper_state_sx, 10)

        self.create_subscription(
            Empty, '/obstacles/clear_all', self.handle_clear_all, 10)
        
        self.create_subscription(
            PoseStamped, '/obstacle_relative', self.handle_obstacle_relative, 10)
        
        # Create publishers
        self.current_obs_pub = self.create_publisher(
            ObstacleArray, '/obstacles/current', 10)
        
        self.path_viz_pub = self.create_publisher(
            MarkerArray, '/path_visualization', 10)
        
        # Create service
        self.path_service = self.create_service(
            ComputePath, '/compute_path', self.handle_compute_path)
        
        self.batch_color_service = self.create_service(
            GetBatchColorPattern, '/get_batch_color_pattern', self.handle_get_batch_color_pattern)
        
        # Create timer for publishing current obstacles
        self.create_timer(1.0 / self.params['rates']['obstacle_list_hz'], 
                         self.publish_current_obstacles)
        
        # Initialize pathfinder
        grid_width = int(self.params['map']['width_mm'] / 
                        self.params['grid']['resolution_mm'])
        grid_height = int(self.params['map']['height_mm'] / 
                         self.params['grid']['resolution_mm'])
        
        effective_width = (self.params['robot']['width_mm'] + 
                          2 * self.params['robot']['safety_margin_mm'])
        effective_length = (self.params['robot']['length_mm'] + 
                           2 * self.params['robot']['safety_margin_mm'])
        
        self.pathfinder = HybridAStarPathfinder(
            grid_width, 
            grid_height,
            effective_width,
            effective_length
        )
        
        # Add permanent obstacle area (0, 600) to (450, 2400)
        self.get_logger().info('🚫 Adding permanent obstacle area...')
        self.pathfinder.add_permanent_obstacle_area(
            x_min=0.0,
            y_min=600.0,
            x_max=450.0,
            y_max=2400.0
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('Hybrid A* Path Planner Node Ready!')
        self.get_logger().info('='*60)
        self.get_logger().info(f"Grid: {grid_width}x{grid_height} @ {self.params['grid']['resolution_mm']}mm")
        self.get_logger().info(f"Robot: {self.params['robot']['width_mm']}x{self.params['robot']['length_mm']}mm")
        self.get_logger().info(f"Safety margin: {self.params['robot']['safety_margin_mm']}mm")
        self.get_logger().info(f"Map: {self.params['map']['width_mm']}x{self.params['map']['height_mm']}mm")
        self.get_logger().info('='*60)
        self.get_logger().info('Waiting for path planning requests...')
    
    def load_parameters(self):
        """Load all ROS2 parameters (replaces JSON loading)"""
        self.params = {
            'robot': {
                'width_mm': self.declare_parameter('robot.width_mm', 350.0).value,
                'length_mm': self.declare_parameter('robot.length_mm', 300.0).value,
                'safety_margin_mm': self.declare_parameter('robot.safety_margin_mm', 10.0).value,
            },
            'grid': {
                'resolution_mm': self.declare_parameter('grid.resolution_mm', 25.0).value,
            },
            'map': {
                'width_mm': self.declare_parameter('map.width_mm', 3000.0).value,
                'height_mm': self.declare_parameter('map.height_mm', 2000.0).value,
            },
            'obstacles': {
                'crate_horizontal': {
                    'width_mm': self.declare_parameter('obstacles.crate_horizontal.width_mm', 150.0).value,
                    'height_mm': self.declare_parameter('obstacles.crate_horizontal.height_mm', 50.0).value,
                },
                'crate_vertical': {
                    'width_mm': self.declare_parameter('obstacles.crate_vertical.width_mm', 50.0).value,
                    'height_mm': self.declare_parameter('obstacles.crate_vertical.height_mm', 150.0).value,
                }
            },
            'pathfinding': {
                'max_iterations': self.declare_parameter('pathfinding.max_iterations', 50000).value,
            },
            'rates': {
                'obstacle_list_hz': self.declare_parameter('rates.obstacle_list_hz', 1.0).value,
            }
        }
    
    def load_static_obstacles(self):
        """Load static wall obstacles from JSON file"""
        import json
        import os
        
        # Try multiple possible paths
        possible_paths = [
            '/home/ubuntu/ROS2-WS-EURO/static_walls.json',
            '/home/ubuntu/hybrid/static_walls.json',
            os.path.join(os.path.dirname(__file__), '../../../static_walls.json'),
            os.path.join(os.path.dirname(__file__), 'static_walls.json'),
        ]
        
        self.get_logger().info('📌 Loading static wall obstacles...')
        
        for json_file in possible_paths:
            if not os.path.exists(json_file):
                continue
            
            try:
                with open(json_file, 'r') as f:
                    data = json.load(f)
                
                static_obs_list = data.get('static_obstacles', [])
                
                for obs_dict in static_obs_list:
                    obs = Obstacle()
                    obs.id = f"static_{obs_dict.get('name', 'wall')}"
                    obs.type = 'wall'  # Special type for walls
                    obs.color = 'wall'  # Special color that can't be filtered
                    obs.pose.x = float(obs_dict.get('x', 0.0))
                    obs.pose.y = float(obs_dict.get('y', 0.0))
                    obs.pose.theta = 0.0
                    obs.width_mm = float(obs_dict.get('width', 50.0))
                    obs.height_mm = float(obs_dict.get('height', 50.0))
                    obs.detected_at = self.get_clock().now().to_msg()
                    
                    self.static_obstacles[obs.id] = obs
                
                self.get_logger().info(f'✅ Loaded {len(self.static_obstacles)} static walls from: {json_file}')
                self.get_logger().info(f'   These walls are ALWAYS active and cannot be ignored')
                return
            
            except Exception as e:
                self.get_logger().error(f'❌ Failed to load static obstacles from {json_file}: {e}')
                continue
        
        self.get_logger().warn('⚠️  No static walls file found - continuing without static obstacles')
    


    def handle_clear_all(self, msg: Empty):
        """Clear all obstacles"""
        with self.obstacles_lock:
            count = len(self.obstacles)
            self.obstacles.clear()
            self.obstacle_sources.clear()
            self.get_logger().info(f"🗑️  Cleared all {count} obstacles")
        # Publish obstacle visualization after clearing
        self.publish_obstacles(self.get_active_obstacles())
    
    def handle_obstacle_relative(self, msg: PoseStamped):
        """Handle incoming obstacle from /obstacle_relative topic (keeps accumulating, doesn't delete)"""
        with self.obstacles_lock:
            # Find and update existing enemy obstacle or add new one
            # We keep all enemy obstacles, just update if position changes significantly
            existing_enemy = None
            for obs in self.obstacles:
                if obs.color == 'enemy':
                    # Check if this is close to an existing enemy (within 200mm)
                    center_x_mm = msg.pose.position.x * 1000.0
                    center_y_mm = msg.pose.position.y * 1000.0
                    obs_center_x = obs.pose.x + (obs.width_mm / 2.0)
                    obs_center_y = obs.pose.y + (obs.height_mm / 2.0)
                    dist = math.sqrt((obs_center_x - center_x_mm)**2 + (obs_center_y - center_y_mm)**2)
                    if dist < 200.0:
                        existing_enemy = obs
                        break
            
            # Convert position from meters to millimeters
            center_x_mm = msg.pose.position.x * 1000.0
            center_y_mm = msg.pose.position.y * 1000.0
            
            if existing_enemy:
                # Update existing enemy obstacle position
                existing_enemy.pose.x = center_x_mm - (existing_enemy.width_mm / 2.0)
                existing_enemy.pose.y = center_y_mm - (existing_enemy.height_mm / 2.0)
                existing_enemy.pose.theta = self.quaternion_to_yaw(msg.pose.orientation)
                existing_enemy.detected_at = msg.header.stamp
                self.get_logger().info(
                    f"🎯 Updated enemy obstacle at ({center_x_mm:.0f}, {center_y_mm:.0f}) mm")
            else:
                # Create new enemy obstacle
                obs = Obstacle()
                obs.id = f"enemy_{int(time.time() * 1000)}"  # Unique ID based on timestamp
                obs.type = 'enemy'
                obs.color = 'enemy'
                
                # Use default enemy obstacle size (can be parameterized if needed)
                # Assuming enemy robot is similar size to our robot
                obs.width_mm = self.params['robot']['width_mm']
                obs.height_mm = self.params['robot']['length_mm']
                
                # Convert to top-left corner (what the planner expects)
                obs.pose.x = center_x_mm - (obs.width_mm / 2.0)
                obs.pose.y = center_y_mm - (obs.height_mm / 2.0)
                obs.pose.theta = self.quaternion_to_yaw(msg.pose.orientation)
                
                obs.detected_at = msg.header.stamp
                
                # Add to obstacles list and track source
                self.obstacles.append(obs)
                self.obstacle_sources[obs.id] = 'enemy'
                self.get_logger().info(
                    f"🎯 Added enemy obstacle at ({center_x_mm:.0f}, {center_y_mm:.0f}) mm")
        
        # Publish obstacle visualization
        self.publish_obstacles(self.get_active_obstacles())

    def publish_current_obstacles(self):
        """Periodically publish current obstacle list"""
        with self.obstacles_lock:
            msg = ObstacleArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.obstacles = list(self.obstacles)
            self.current_obs_pub.publish(msg)
        # Also publish obstacle visualization for RViz
        self.publish_obstacles(self.get_active_obstacles())

    def quaternion_to_yaw(self, quat: Quaternion) -> float:
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat
    
    def get_active_obstacles(self) -> List[dict]:
        """Get obstacles filtered by color (permanent areas are ALWAYS included in pathfinder)"""
        active_obstacles = []
        
        # NOTE: Permanent obstacle areas are automatically re-applied in pathfinder.set_obstacles()
        # No need to add them here as they're handled internally by the pathfinder
        
        # Add dynamic obstacles (filtered by color)
        with self.obstacles_lock:
            for obs in self.obstacles:
                active_obstacles.append({
                    'x': obs.pose.x,
                    'y': obs.pose.y,
                    'width': obs.width_mm,
                    'height': obs.height_mm,
                    'theta': obs.pose.theta,
                    'id': obs.id,
                    'color': obs.color
                })
        
        return active_obstacles
    
    def convert_to_path_msg(self, waypoints: List[Tuple[float, float, float]], 
                           header: Header) -> Path:
        """Convert waypoint list to nav_msgs/Path"""
        path_msg = Path()
        path_msg.header = header
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y, theta in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = x / 1000.0  # Convert mm to m
            pose_stamped.pose.position.y = y / 1000.0  # Convert mm to m
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation = self.yaw_to_quaternion(theta)
            path_msg.poses.append(pose_stamped)
        
        return path_msg

    def publish_obstacles(self, obstacles: List[dict]):
        # Separate walls from crates
        #self.get_logger().info("Publishing obstacle visualization...")
        marker_array = MarkerArray()
        
        # First, send a DELETE ALL marker to clear old markers
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "crates"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        crates = [obs for obs in obstacles if obs['color'] != 'wall']
        
        marker_id = 1
        # Visualize CRATES (small, colored, on ground)
        for obs in crates:
            crate_marker = Marker()
            crate_marker.header.frame_id = "map"
            crate_marker.header.stamp = self.get_clock().now().to_msg()
            crate_marker.ns = "crates"
            crate_marker.id = marker_id
            marker_id += 1
            crate_marker.type = Marker.CUBE
            crate_marker.action = Marker.ADD
            
            # Position (on ground level)
            crate_marker.pose.position.x = (obs['x'] + (obs['width'] / 2.0))/ 1000.0
            crate_marker.pose.position.y = (obs['y'] +(obs['height'] / 2.0))/ 1000.0
            crate_marker.pose.position.z = 0.03  # 30mm high (half of 150mm)
            crate_marker.pose.orientation = self.yaw_to_quaternion(obs.get('theta', 0.0))
            
            # Size: small crates
            crate_marker.scale.x = obs['width'] / 1000.0
            crate_marker.scale.y = obs['height'] / 1000.0
            crate_marker.scale.z = 0.15  # 150mm tall crates
            
            # Color based on crate color
            if obs['color'] == 'yellow':
                crate_marker.color.r = 1.0
                crate_marker.color.g = 1.0
                crate_marker.color.b = 0.0
                crate_marker.color.a = 0.8
            elif obs['color'] == 'blue':
                crate_marker.color.r = 0.0
                crate_marker.color.g = 0.5
                crate_marker.color.b = 1.0
                crate_marker.color.a = 0.8
            elif obs['color'] == 'enemy':
                # Dark red/magenta for enemy obstacles
                crate_marker.color.r = 0.8
                crate_marker.color.g = 0.0
                crate_marker.color.b = 0.4
                crate_marker.color.a = 0.9
            else:  # red or other
                crate_marker.color.r = 1.0
                crate_marker.color.g = 0.0
                crate_marker.color.b = 0.0
                crate_marker.color.a = 0.8
            
            marker_array.markers.append(crate_marker)
        
        self.path_viz_pub.publish(marker_array)
    
    def handle_aruco_mapper_state_fx(self, msg: ArucoMapperState):
        """
        Handle ArUco mapper state updates from FX side
        Extract obstacles from batches and update obstacle database
        Also store batches for color pattern queries
        """
        self._handle_aruco_mapper_state(msg, 'fx')
    
    def handle_aruco_mapper_state_sx(self, msg: ArucoMapperState):
        """
        Handle ArUco mapper state updates from SX side
        Extract obstacles from batches and update obstacle database
        Also store batches for color pattern queries
        """
        self._handle_aruco_mapper_state(msg, 'sx')
    
    def _handle_aruco_mapper_state(self, msg: ArucoMapperState, source: str):
        """
        Internal method to handle ArUco mapper state updates
        source: 'fx' or 'sx'
        """
        self.get_logger().info(f"🔔 Received ArUco mapper state ({source.upper()}) with {len(msg.batches)} batches")
        with self.obstacles_lock:
            # Remove only obstacles from this source
            old_count = len(self.obstacles)
            self.obstacles = [obs for obs in self.obstacles if self.obstacle_sources.get(obs.id, '') != source]
            removed = old_count - len(self.obstacles)
            
            # Clear obstacle sources for this source
            self.obstacle_sources = {k: v for k, v in self.obstacle_sources.items() if v != source}
            
            if removed > 0:
                self.get_logger().info(f"🗑️  Removed {removed} old {source.upper()} obstacles")

            # Process each batch
            for batch in msg.batches:
                # Store batch by ID in the appropriate dictionary
                with self.batches_lock:
                    if source == 'fx':
                        self.batches_fx[batch.batch_id] = batch
                    else:
                        self.batches_sx[batch.batch_id] = batch

                if not batch.members:
                    continue
                
                if source == 'sx':
                    # SX: Use predefined batch position from lookup table
                    batch_known_id = batch.known_id
                    
                    if batch_known_id not in self.predefined_batch_positions:
                        self.get_logger().warn(f"⚠️  Batch '{batch_known_id}' not found in predefined positions, skipping")
                        continue
                    
                    batch_pos = self.predefined_batch_positions[batch_known_id]
                    batch_center_x = batch_pos["x"]  # Already in meters
                    batch_center_y = batch_pos["y"]  # Already in meters
                    
                    # Convert batch center from meters to millimeters
                    batch_center_x_mm = batch_center_x * 1000.0
                    batch_center_y_mm = batch_center_y * 1000.0
                    
                    # Determine obstacle dimensions based on batch direction
                    # Each obstacle is 15x5 cm or 5x15 cm depending on orientation
                    if batch.direction == "horizontal":
                        obs_width_mm = 50.0   # 5 cm (narrow)
                        obs_height_mm = 150.0 # 15 cm (tall)
                        # For horizontal: 4 obstacles side by side (total width = 200mm)
                        # Centers at: cx-75, cx-25, cx+25, cx+75 (edge to edge)
                        offsets = [
                            (-75.0, 0.0),  # Left-most
                            (-25.0, 0.0),
                            (25.0, 0.0),
                            (75.0, 0.0)    # Right-most
                        ]
                    elif batch.direction == "vertical":
                        obs_width_mm = 150.0  # 15 cm (wide)
                        obs_height_mm = 50.0  # 5 cm (short)
                        # For vertical: 4 obstacles stacked (total height = 200mm)
                        # Centers at: cy-75, cy-25, cy+25, cy+75 (edge to edge)
                        offsets = [
                            (0.0, -75.0),  # Bottom-most
                            (0.0, -25.0),
                            (0.0, 25.0),
                            (0.0, 75.0)    # Top-most
                        ]
                    else:
                        # Default to horizontal if direction is unknown
                        obs_width_mm = 50.0
                        obs_height_mm = 150.0
                        offsets = [(-75.0, 0.0), (-25.0, 0.0), (25.0, 0.0), (75.0, 0.0)]
                    
                    # Get colors from members in their original order (up to 4)
                    member_colors = [m.color for m in batch.members if m.color in ["yellow", "blue"]]
                    if not member_colors:
                        continue  # Skip batch if no valid colors
                    
                    # Reverse the color order
                    member_colors.reverse()
                    
                    # Create 4 obstacles for this batch
                    for idx, (offset_x, offset_y) in enumerate(offsets):
                        obs = Obstacle()
                        obs.id = f"{source}_batch{batch.batch_id}_member{idx}"
                        obs.type = 'crate'
                        # Use color from member at this index (or last color if not enough members)
                        obs.color = member_colors[idx] if idx < len(member_colors) else member_colors[-1]
                        obs.width_mm = obs_width_mm
                        obs.height_mm = obs_height_mm
                        
                        # Calculate obstacle center
                        obs_center_x = batch_center_x_mm + offset_x
                        obs_center_y = batch_center_y_mm + offset_y
                        
                        # Convert to TOP-LEFT corner (what the planner expects)
                        obs.pose.x = obs_center_x - (obs_width_mm / 2.0)
                        obs.pose.y = obs_center_y - (obs_height_mm / 2.0)
                        obs.pose.theta = 0.0
                        
                        obs.detected_at = msg.stamp
                        
                        # Add to obstacles list and track source
                        self.obstacles.append(obs)
                        self.obstacle_sources[obs.id] = source
                
                else:
                    # FX: Use individual member positions
                    for member in batch.members:
                        # Only process if color is valid
                        if member.color not in ["yellow", "blue"]:
                            continue

                        # Create obstacle from batch member
                        obs = Obstacle()
                        obs.id = f"aruco_{member.aruco_id}"
                        obs.type = 'crate'
                        obs.color = member.color

                        # Determine obstacle dimensions based on batch direction (swapped)
                        if batch.direction == "horizontal":
                            obs.width_mm = self.params['obstacles']['crate_horizontal']['height_mm']
                            obs.height_mm = self.params['obstacles']['crate_horizontal']['width_mm']
                        elif batch.direction == "vertical":
                            obs.width_mm = self.params['obstacles']['crate_vertical']['height_mm']
                            obs.height_mm = self.params['obstacles']['crate_vertical']['width_mm']
                        else:
                            # Default to horizontal if direction is unknown
                            obs.width_mm = self.params['obstacles']['crate_horizontal']['height_mm']
                            obs.height_mm = self.params['obstacles']['crate_horizontal']['width_mm']

                        # IMPORTANT: map_pos is the CENTER of the obstacle
                        # Convert to TOP-LEFT corner (what the planner expects)
                        # Convert from meters to millimeters
                        center_x_mm = member.map_pos.x * 1000.0 
                        center_y_mm = member.map_pos.y * 1000.0
                        obs.pose.x = center_x_mm - (obs.width_mm / 2.0)   # Center to top-left
                        obs.pose.y = center_y_mm - (obs.height_mm / 2.0)  # Center to top-left
                        obs.pose.theta = 0.0

                        obs.detected_at = msg.stamp

                        # Add to obstacles list and track source
                        self.obstacles.append(obs)
                        self.obstacle_sources[obs.id] = source
            
            # Count obstacles by source
            fx_count = sum(1 for v in self.obstacle_sources.values() if v == 'fx')
            sx_count = sum(1 for v in self.obstacle_sources.values() if v == 'sx')
            enemy_count = sum(1 for v in self.obstacle_sources.values() if v == 'enemy')
            
            self.get_logger().info(
                f"📦 Updated {source.upper()} obstacles from {len(msg.batches)} batches | "
                f"Total obstacles: FX={fx_count}, SX={sx_count}, Enemy={enemy_count}")
    def publish_path_visualization(self, waypoints: List[Tuple[float, float, float]], 
                                   obstacles: List[dict]):
        """Publish path and obstacles visualization for RViz"""
        marker_array = MarkerArray()
        
        # Path line marker
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05  # Line width
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0
        
        for x, y, _ in waypoints:
            point = path_marker.points.add() if hasattr(path_marker.points, 'add') else None
            if point is None:
                from geometry_msgs.msg import Point
                point = Point()
                path_marker.points.append(point)
            point.x = x / 1000.0
            point.y = y / 1000.0
            point.z = 0.0
        
        marker_array.markers.append(path_marker)
        
        # Separate walls from crates
        crates = [obs for obs in obstacles if obs['color'] != 'wall']
        
        marker_id = 1
        # Visualize CRATES (small, colored, on ground)
        for obs in crates:
            crate_marker = Marker()
            crate_marker.header.frame_id = "map"
            crate_marker.header.stamp = self.get_clock().now().to_msg()
            crate_marker.ns = "crates"
            crate_marker.id = marker_id
            marker_id += 1
            crate_marker.type = Marker.CUBE
            crate_marker.action = Marker.ADD
            
            # Position (on ground level)
            crate_marker.pose.position.x = (obs['x'] + (obs['width'] / 2.0))/ 1000.0
            crate_marker.pose.position.y = (obs['y'] +(obs['height'] / 2.0))/ 1000.0
            crate_marker.pose.position.z = 0.03  # 30mm high (half of 150mm)
            crate_marker.pose.orientation = self.yaw_to_quaternion(obs.get('theta', 0.0))
            
            # Size: small crates
            crate_marker.scale.x = obs['width'] / 1000.0
            crate_marker.scale.y = obs['height'] / 1000.0
            crate_marker.scale.z = 0.15  # 150mm tall crates
            
            # Color based on crate color
            if obs['color'] == 'yellow':
                crate_marker.color.r = 1.0
                crate_marker.color.g = 1.0
                crate_marker.color.b = 0.0
                crate_marker.color.a = 0.8
            elif obs['color'] == 'blue':
                crate_marker.color.r = 0.0
                crate_marker.color.g = 0.5
                crate_marker.color.b = 1.0
                crate_marker.color.a = 0.8
            else:  # red or other
                crate_marker.color.r = 1.0
                crate_marker.color.g = 0.0
                crate_marker.color.b = 0.0
                crate_marker.color.a = 0.8
            
            marker_array.markers.append(crate_marker)
        
        self.path_viz_pub.publish(marker_array)
    
    def optimize_path_with_intersections(
    self,
    path: list,
    grid_resolution: float,
    max_added_distance_mm: float = 0.0,  # your threshold
    sample_step_mm: float = 50.0
) -> list:
        """
        Simplify a path by replacing 4-point zig-zag segments with their line
        intersection when it is collision-free and does not add too much distance.

        Input path format:
            [(x, y, theta), ...]

        Returns:
            Optimized path [(x, y, theta), ...]
        """

        if len(path) < 4:
            return path

        def collision_check_segment(p1, p2, theta):
            """
            Check collision along a straight segment by sampling points
            """
            dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            steps = max(5, int(dist / sample_step_mm))
            for i in range(steps + 1):
                t = i / steps
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])
                if self.pathfinder.is_collision(x, y, theta, grid_resolution):
                    return True
            return False

        def segment_distance(p1, p2):
            return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

        optimized = path.copy()
        index = 0
        removed = 0

        while index <= len(optimized) - 4:
            x1, y1, t1 = optimized[index]
            x2, y2, _  = optimized[index + 1]
            x3, y3, _  = optimized[index + 2]
            x4, y4, t4 = optimized[index + 3]

            # Compute intersection of (p1,p2) with (p3,p4)
            den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if abs(den) < 1e-6:
                index += 1
                continue

            px = ((x1 * y2 - y1 * x2) * (x3 - x4) -
                (x1 - x2) * (x3 * y4 - y3 * x4)) / den
            py = ((x1 * y2 - y1 * x2) * (y3 - y4) -
                (y1 - y2) * (x3 * y4 - y3 * x4)) / den

            intersection = (px, py)

            # Orientation from previous to intersection
            new_theta = math.atan2(py - y1, px - x1)

            # Collision checks
            collision = (
                collision_check_segment((x1, y1), intersection, new_theta) or
                collision_check_segment(intersection, (x4, y4), t4)
            )

            # Calculate distance difference if we replace p2 + p3 with intersection
            old_dist = segment_distance((x1, y1), (x2, y2)) + segment_distance((x2, y2), (x3, y3)) + segment_distance((x3, y3), (x4, y4))
            new_dist = segment_distance((x1, y1), intersection) + segment_distance(intersection, (x4, y4))
            added_distance = new_dist - old_dist

            if not collision and added_distance <= max_added_distance_mm:
                # Accept simplification
                optimized[index + 1] = (px, py, new_theta)
                del optimized[index + 2]
                removed += 1
                continue  # re-check same index
            else:
                # Skip this simplification due to collision or too much added distance
                index += 1

        if removed > 0:
            self.get_logger().info(f"✨ Intersection optimization removed {removed} waypoints")

        return optimized
    def handle_compute_path(self, request: ComputePath.Request, 
                           response: ComputePath.Response) -> ComputePath.Response:
        """
        Service callback: compute path from start to goal
        Uses the same advanced pathfinding logic as the local test script.
        """
        start_time = time.time()
        
        # Extract start and goal (convert m to mm)
        start_x = request.start_pose.pose.position.x * 1000.0
        start_y = request.start_pose.pose.position.y * 1000.0
        start_theta = self.quaternion_to_yaw(request.start_pose.pose.orientation)
        
        goal_x = request.goal_pose.pose.position.x * 1000.0
        goal_y = request.goal_pose.pose.position.y * 1000.0
        goal_theta = self.quaternion_to_yaw(request.goal_pose.pose.orientation)
        
        self.get_logger().info('='*60)
        self.get_logger().info(f"🔍 Computing path:")
        self.get_logger().info(f"   Start: ({start_x:.0f}, {start_y:.0f}) θ={math.degrees(start_theta):.1f}° (rad={start_theta:.3f})")
        self.get_logger().info(f"   Goal:  ({goal_x:.0f}, {goal_y:.0f}) θ={math.degrees(goal_theta):.1f}° (rad={goal_theta:.3f})")
        
        # Filter obstacles by color
        active_obstacles = self.get_active_obstacles()
        
        self.get_logger().info(f"   Dynamic obstacles: {len(active_obstacles)}")
        self.get_logger().info(f"   Permanent area: (0,600)→(450,2400) always blocked")
        
        try:
            # Set obstacles in pathfinder
            grid_resolution = self.params['grid']['resolution_mm']
            
            # All obstacles are dynamic (permanent area is handled automatically)
            self.pathfinder.set_obstacles(active_obstacles, grid_resolution, static_walls=None)
            
            # Run Hybrid A* algorithm (same as local script)
            self.get_logger().info("🔍 Searching for path...")
            path = self.pathfinder.find_path(
                start_pose=(start_x, start_y, start_theta),
                goal_pose=(goal_x, goal_y, goal_theta),
                grid_resolution=grid_resolution,
                max_iterations=self.params['pathfinding']['max_iterations']
            )
            
            if path:
                self.get_logger().info(f"✅ Found path: {len(path)} waypoints")
                
                # Smooth path (always enabled, same as local script)
                smoothed = self.pathfinder.smooth_path(path, grid_resolution)
                reduction = 100 * (1 - len(smoothed) / len(path)) if len(path) > 0 else 0
                self.get_logger().info(f"🔄 Smoothed to {len(smoothed)} waypoints ({reduction:.1f}% reduction)")
                
                # Try to set the goal orientation if it's collision-free
                if len(smoothed) > 0:
                    x, y, current_theta = smoothed[-1]
                    
                    self.get_logger().info(f"  Checking goal rotation at ({x:.1f}, {y:.1f}): {math.degrees(current_theta):.1f}° → {math.degrees(goal_theta):.1f}°")
                    
                    # Check multiple intermediate angles during rotation to ensure safe rotation
                    angle_diff = goal_theta - current_theta
                    # Normalize to [-pi, pi]
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                    
                    # Check if rotation path is collision-free
                    rotation_safe = True
                    num_rotation_checks = max(18, int(abs(angle_diff) / math.radians(5)))  # Check every ~5°
                    for i in range(num_rotation_checks + 1):
                        ratio = i / num_rotation_checks
                        check_angle = current_theta + ratio * angle_diff
                        if self.pathfinder.is_collision(x, y, check_angle, grid_resolution):
                            self.get_logger().info(f"    ⚠ Rotation collision at angle {math.degrees(check_angle):.1f}° (step {i}/{num_rotation_checks})")
                            rotation_safe = False
                            break
                    
                    if rotation_safe:
                        # Add intermediate waypoint for rotation if angle change is large
                        if abs(angle_diff) > math.radians(30):
                            # Keep position but change orientation gradually
                            smoothed.append((x, y, goal_theta))
                            self.get_logger().info(f"✓ Added rotation waypoint: {math.degrees(current_theta):.1f}° → {math.degrees(goal_theta):.1f}°")
                        else:
                            smoothed[-1] = (x, y, goal_theta)
                            self.get_logger().info(f"✓ Final orientation set to goal: {math.degrees(goal_theta):.1f}°")
                    else:
                        self.get_logger().info(f"⚠ Goal orientation {math.degrees(goal_theta):.1f}° would cause collision during rotation, keeping {math.degrees(current_theta):.1f}°")
                
                # Validate that each waypoint can safely rotate to travel angle
                self.get_logger().info(f"🔍 Validating travel rotations for {len(smoothed)} waypoints...")
                validated = []
                for i in range(len(smoothed)):
                    x, y, theta = smoothed[i]
                    
                    if i < len(smoothed) - 1:
                        # Calculate travel angle toward next waypoint
                        next_x, next_y, _ = smoothed[i+1]
                        travel_angle = math.atan2(next_y - y, next_x - x)
                        
                        # Check if robot can rotate from current theta to travel angle
                        angle_diff = travel_angle - theta
                        while angle_diff > math.pi:
                            angle_diff -= 2 * math.pi
                        while angle_diff < -math.pi:
                            angle_diff += 2 * math.pi
                        
                        # Check intermediate angles during rotation
                        num_checks = max(10, int(abs(angle_diff) / math.radians(5)))
                        collision_found = False
                        for j in range(num_checks + 1):
                            ratio = j / num_checks
                            check_angle = theta + ratio * angle_diff
                            if self.pathfinder.is_collision(x, y, check_angle, grid_resolution):
                                collision_found = True
                                break
                        
                        if collision_found:
                            # Try to find a safe orientation at this position
                            safe_angle = None
                            for test_deg in range(-180, 180, 5):
                                test_angle = math.radians(test_deg)
                                # Check if this angle is collision-free
                                if not self.pathfinder.is_collision(x, y, test_angle, grid_resolution):
                                    # Check if we can rotate to travel angle from this angle
                                    test_diff = travel_angle - test_angle
                                    while test_diff > math.pi:
                                        test_diff -= 2 * math.pi
                                    while test_diff < -math.pi:
                                        test_diff += 2 * math.pi
                                    
                                    test_safe = True
                                    test_checks = max(10, int(abs(test_diff) / math.radians(5)))
                                    for k in range(test_checks + 1):
                                        ratio = k / test_checks
                                        check_angle = test_angle + ratio * test_diff
                                        if self.pathfinder.is_collision(x, y, check_angle, grid_resolution):
                                            test_safe = False
                                            break
                                    
                                    if test_safe:
                                        safe_angle = test_angle
                                        break
                            
                            if safe_angle is not None:
                                validated.append((x, y, safe_angle))
                            else:
                                validated.append((x, y, theta))
                        else:
                            validated.append((x, y, theta))
                    else:
                        # Last waypoint, keep as is
                        validated.append((x, y, theta))
                
                smoothed = validated
                
                # Remove waypoints that are too close (rotation-only waypoints)
                self.get_logger().info(f"🔧 Removing rotation-only waypoints...")
                optimized = [smoothed[0]]  # Always keep first waypoint
                
                i = 1
                while i < len(smoothed) - 1:
                    prev_x, prev_y, prev_theta = optimized[-1]  # Use last kept waypoint
                    curr_x, curr_y, curr_theta = smoothed[i]
                    next_x, next_y, next_theta = smoothed[i+1]
                    
                    # Calculate distance from last kept waypoint to current
                    dist_to_curr = math.sqrt((curr_x - prev_x)**2 + (curr_y - prev_y)**2)
                    
                    # If waypoint is very close (< 100mm), it's just a rotation point
                    if dist_to_curr < 100.0:
                        i += 1
                        continue
                    
                    # Calculate angle from last kept waypoint to current
                    angle_to_curr = math.atan2(curr_y - prev_y, curr_x - prev_x)
                    # Calculate angle from current to next
                    angle_to_next = math.atan2(next_y - curr_y, next_x - curr_x)
                    
                    # Check if the path is roughly straight (angles are similar)
                    angle_diff = abs(math.atan2(math.sin(angle_to_next - angle_to_curr), 
                                               math.cos(angle_to_next - angle_to_curr)))
                    
                    # Check if orientations are similar between current and next
                    theta_diff = abs(math.atan2(math.sin(next_theta - curr_theta), 
                                               math.cos(next_theta - curr_theta)))
                    
                    # Keep waypoint if path changes significantly
                    if angle_diff > math.radians(15) or theta_diff > math.radians(20):
                        optimized.append(smoothed[i])
                    
                    i += 1
                
                if len(smoothed) > 0:
                    optimized.append(smoothed[-1])  # Always keep last waypoint
                
                if len(optimized) < len(smoothed):
                    self.get_logger().info(f"🎯 Optimized to {len(optimized)} waypoints (removed {len(smoothed)-len(optimized)} rotation-only waypoints)")
                    smoothed = optimized
                
                self.get_logger().info(f"ℹ️  Intersection optimization disabled - keeping all waypoints")
                self.get_logger().info(f"ℹ️  Axis-aligned projection disabled - keeping all waypoints")
                
                # FINAL VALIDATION: Check each waypoint can safely rotate to travel toward next waypoint
                self.get_logger().info(f"🔒 Final safety check: validating travel rotations...")
                final_path = []
                for i in range(len(smoothed)):
                    x, y, theta = smoothed[i]
                    
                    if i < len(smoothed) - 1:
                        # Calculate travel angle toward next waypoint
                        next_x, next_y, _ = smoothed[i+1]
                        travel_angle = math.atan2(next_y - y, next_x - x)
                        
                        # Check if robot can safely rotate from theta to travel_angle
                        angle_diff = travel_angle - theta
                        while angle_diff > math.pi:
                            angle_diff -= 2 * math.pi
                        while angle_diff < -math.pi:
                            angle_diff += 2 * math.pi
                        
                        # Check intermediate angles during rotation
                        num_checks = max(18, int(abs(angle_diff) / math.radians(5)))
                        collision_found = False
                        for j in range(num_checks + 1):
                            ratio = j / num_checks
                            check_angle = theta + ratio * angle_diff
                            if self.pathfinder.is_collision(x, y, check_angle, grid_resolution):
                                collision_found = True
                                break
                        
                        if collision_found:
                            # Try to find a safe orientation
                            safe_angle = None
                            for test_deg in range(-180, 180, 5):
                                test_angle = math.radians(test_deg)
                                if self.pathfinder.is_collision(x, y, test_angle, grid_resolution):
                                    continue
                                # Check if we can rotate to travel angle from this angle
                                test_diff = travel_angle - test_angle
                                while test_diff > math.pi:
                                    test_diff -= 2 * math.pi
                                while test_diff < -math.pi:
                                    test_diff += 2 * math.pi
                                
                                test_safe = True
                                test_checks = max(18, int(abs(test_diff) / math.radians(5)))
                                for k in range(test_checks + 1):
                                    ratio = k / test_checks
                                    check_angle = test_angle + ratio * test_diff
                                    if self.pathfinder.is_collision(x, y, check_angle, grid_resolution):
                                        test_safe = False
                                        break
                                
                                if test_safe:
                                    safe_angle = test_angle
                                    break
                            
                            if safe_angle is not None:
                                final_path.append((x, y, safe_angle))
                            else:
                                # Try to find safe intermediate position
                                found_safe = False
                                for dist_ratio in [0.2, 0.4, 0.6, 0.8]:
                                    mid_x = x + (next_x - x) * dist_ratio
                                    mid_y = y + (next_y - y) * dist_ratio
                                    mid_to_next_angle = math.atan2(next_y - mid_y, next_x - mid_x)
                                    
                                    if not self.pathfinder.is_collision(mid_x, mid_y, mid_to_next_angle, grid_resolution):
                                        curr_to_mid_angle = math.atan2(mid_y - y, mid_x - x)
                                        if not self.pathfinder.is_collision(x, y, curr_to_mid_angle, grid_resolution):
                                            final_path.append((x, y, curr_to_mid_angle))
                                            final_path.append((mid_x, mid_y, mid_to_next_angle))
                                            found_safe = True
                                            break
                                
                                if not found_safe:
                                    final_path.append((x, y, theta))
                        else:
                            final_path.append((x, y, theta))
                    else:
                        # Last waypoint
                        final_path.append((x, y, theta))
                
                smoothed = final_path
                self.get_logger().info("🧠 Applying intersection optimization...")
                smoothed = self.optimize_path_with_intersections(smoothed, grid_resolution=grid_resolution)
                # Calculate path length
                total_dist = 0
                for i in range(len(smoothed) - 1):
                    x1, y1, _ = smoothed[i]
                    x2, y2, _ = smoothed[i + 1]
                    total_dist += math.sqrt((x2-x1)**2 + (y2-y1)**2)
                
                self.get_logger().info(f"📏 Path length: {total_dist:.1f}mm ({total_dist/1000:.2f}m)")
                
                # Log waypoint summary (same as local script)
                self.get_logger().info(f"📍 Path waypoints:")
                for i, (x, y, theta) in enumerate(smoothed):
                    if i < 3 or i >= len(smoothed) - 2:
                        self.get_logger().info(f"   {i+1:3d}. ({x:7.1f}, {y:7.1f}) θ={math.degrees(theta):6.1f}°")
                    elif i == 3:
                        self.get_logger().info(f"   ... ({len(smoothed)-5} waypoints omitted)")
                
                # Convert to ROS2 Path message
                response.path = self.convert_to_path_msg(smoothed, request.start_pose.header)
                response.success = True
                response.num_waypoints = len(smoothed)
                response.path_length_m = total_dist / 1000.0
                response.message = f"Path found: {len(smoothed)} waypoints, {response.path_length_m:.2f}m"
                
                # Add STM32 go_to commands to response
                response.stm32_commands = self.waypoints_to_stm32_go_to_commands(smoothed, speed=100)
                
                # Publish visualization
                self.publish_path_visualization(smoothed, active_obstacles)
                
                self.get_logger().info(f"✅ Success!")
            else:
                response.success = False
                response.message = "No path found - start/goal in collision or blocked"
                self.get_logger().warn(f"❌ {response.message}")
                self.get_logger().warn("Possible issues:")
                self.get_logger().warn("  - Start or goal in collision with obstacles")
                self.get_logger().warn("  - Path blocked by obstacles")
                self.get_logger().warn("  - Try adjusting start/goal positions")
        
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(f"❌ Planning failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        finally:
            response.planning_time_ms = (time.time() - start_time) * 1000.0
            response.num_obstacles_used = len(active_obstacles)
            self.get_logger().info(f"   Planning time: {response.planning_time_ms:.1f}ms")
            self.get_logger().info('='*60)
        return response
    def handle_get_batch_color_pattern(self, request, response):
        """
        Service handler: Get color pattern for a batch by known_id or position
        If known_id is empty, finds closest batch to the provided position
        """

        known_id = request.batch_known_id.strip()

        with self.batches_lock:
            batch = None
            
            # Combine batches from both fx and sx
            all_batches = {**self.batches_fx, **self.batches_sx}
            
            # If known_id is empty, find closest batch to position
            if known_id == "":
                if not all_batches:
                    response.success = False
                    response.color_pattern = ""
                    response.direction = ""
                    response.batch_center.x = 0.0
                    response.batch_center.y = 0.0
                    response.batch_center.z = 0.0
                    response.message = "No batches available"
                    self.get_logger().warn("❌ No batches available for position-based search")
                    return response
                
                # Find closest batch to the provided position
                pos_x = request.pos.x
                pos_y = request.pos.y
                min_distance = float('inf')
                closest_batch = None
                
                for b in all_batches.values():
                    if not b.members:
                        continue
                    
                    # Calculate center of batch
                    avg_x = sum(m.map_pos.x for m in b.members) / len(b.members)
                    avg_y = sum(m.map_pos.y for m in b.members) / len(b.members)
                    
                    # Calculate distance
                    distance = math.sqrt((avg_x - pos_x)**2 + (avg_y - pos_y)**2)
                    
                    if distance < min_distance:
                        min_distance = distance
                        closest_batch = b
                
                if closest_batch is None:
                    response.success = False
                    response.color_pattern = ""
                    response.direction = ""
                    response.batch_center.x = 0.0
                    response.batch_center.y = 0.0
                    response.batch_center.z = 0.0
                    response.message = "No valid batches found"
                    self.get_logger().warn("❌ No valid batches found for position search")
                    return response
                
                batch = closest_batch
                self.get_logger().info(
                    f"📍 Found closest batch '{batch.known_id}' at distance {min_distance:.1f}mm "
                    f"from position ({pos_x:.1f}, {pos_y:.1f})"
                )
            else:
                # Find batch by known_id
                for b in all_batches.values():
                    if b.known_id == known_id:
                        batch = b
                        break

                if batch is None:
                    response.success = False
                    response.color_pattern = ""
                    response.direction = ""
                    response.batch_center.x = 0.0
                    response.batch_center.y = 0.0
                    response.batch_center.z = 0.0
                    response.message = f"Batch with known_id '{known_id}' not found"
                    self.get_logger().warn(f"❌ Batch known_id '{known_id}' not found")
                    return response

            # Calculate batch center position
            if batch.members:
                center_x = sum(m.map_pos.x for m in batch.members) / len(batch.members)
                center_y = sum(m.map_pos.y for m in batch.members) / len(batch.members)
            else:
                center_x = 0.0
                center_y = 0.0
            
            # Set batch center in response
            response.batch_center.x = center_x
            response.batch_center.y = center_y
            response.batch_center.z = 0.0

            # Direction
            direction = batch.direction.lower()
            response.direction = direction

            # Copy members
            members = list(batch.members)

            # Sort members based on direction
            if direction == "vertical":
                # bottom → top, right → left
                members.sort(
                    key=lambda m: (m.map_pos.y, m.map_pos.x),
                    reverse=True
                )
            else:  # horizontal
                # right → left, top → bottom
                members.sort(
                    key=lambda m: (m.map_pos.x, m.map_pos.y),
                    reverse=True
                )

            # Build color pattern
            color_pattern = ""
            for member in members:
                if member.color == "yellow":
                    color_pattern += "y"
                elif member.color == "blue":
                    color_pattern += "b"
                else:
                    color_pattern += "?"

            response.success = True
            response.color_pattern = color_pattern
            
            if known_id == "":
                response.message = (
                    f"Closest batch '{batch.known_id}' ({direction}): {color_pattern}"
                )
            else:
                response.message = (
                    f"Batch '{known_id}' ({direction}): {color_pattern}"
                )

            self.get_logger().info(
                f"📋 Batch '{batch.known_id}' → pattern='{color_pattern}', "
                f"direction={direction}, members={len(members)}, "
                f"center=({center_x:.1f}, {center_y:.1f})"
            )

            return response
    def waypoints_to_stm32_go_to_commands(self, waypoints, speed=100):
        """
        Convert a list of (x, y, theta) waypoints to STM32 go_to(x, y, speed) commands.
        Each go_to means: rotate to face (x, y), then move forward to (x, y) at the given speed.
        x, y are in mm.
        Returns a list of string commands.
        """
        commands = []
        if not waypoints or len(waypoints) < 2:
            return commands
        for i in range(1, len(waypoints)):
            x, y, _ = waypoints[i]
            commands.append(f"go_to({int(round(x))}, {int(round(y))}, {int(speed)})")
        return commands
    
def main(args=None):
    rclpy.init(args=args)
    node = HybridAStarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()