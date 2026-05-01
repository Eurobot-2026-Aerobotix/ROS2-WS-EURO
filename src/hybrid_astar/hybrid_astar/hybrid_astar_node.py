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
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

# Custom messages (will be generated)
from hybrid_astar.msg import Obstacle, ObstacleArray, ColorCommand, ActiveBatches
from aruco_mapper_msgs.msg import ArucoMapperState, Batch, BatchMember
from aruco_mapper_msgs.srv import GetStableBatches
from hybrid_astar.srv import ComputePath, GetBatchColorPattern, DeleteBatch, GetActiveBatches, PutDown, SetTeamColor

# Import the pathfinding algorithm
from hybrid_astar.hybrid_astar import HybridAStarPathfinder
from hybrid_astar.map import MapManager

class HybridAStarNode(Node):
    """
    ROS2 node for Hybrid A* pathfinding with dynamic obstacle management.
    
    Subscribes to:
        - /obstacles/clear_all: Clear all obstacles
        - /obstacle_relative: Receive enemy robot poses as obstacles (PoseStamped)
    
    Publishes:
        - /obstacles/current: Current obstacle list
        - /path_visualization: Path visualization for RViz
    
    Services:
        - /compute_path: Compute path from start to goal
        - /put_down: Add 4 placed crate obstacles in front/back of robot
        - /get_batch_color_pattern: Get color pattern for a batch
        - /update_map: Trigger to fetch stable batches from ArUco mapper
    
    Clients:
        - /aruco_mapper/get_stable_batches: Request stable batches from ArUco mapper
    """
    
    def __init__(self):
        super().__init__('hybrid_astar_node')
        
        # Log deduplication - track last N messages to prevent spam
        self._last_log_messages = []
        self._max_log_history = 5
        self._last_active_batches_signature = None
        
        # Load all parameters
        self.load_parameters()
        
        # Separate callback groups to allow true parallel execution between
        # path planning and map-management flows on MultiThreadedExecutor.
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.path_callback_group = ReentrantCallbackGroup()
        self.map_callback_group = ReentrantCallbackGroup()
        self.misc_callback_group = ReentrantCallbackGroup()

        # TF for current robot pose lookup (map -> base_link/base_footprint)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_robot_pose_mm: Optional[Tuple[float, float, float]] = None
        
        # Create clients for GetStableBatches service on map callback group
        # 1. Web handler service (for pathfinding simulation batches)
        self.get_stable_batches_client_web = self.create_client(
            GetStableBatches, '/aruco_mapper/get_stable_batches_web',
            callback_group=self.map_callback_group)
        
        # 2. ArUco mapper service (for real robot batches)
        self.get_stable_batches_client = self.create_client(
            GetStableBatches, '/aruco_mapper/get_stable_batches',
            callback_group=self.map_callback_group)
        
        # Initialize map manager (handles obstacles, batches, and topics)
        # Pass both clients - web client has priority
        self.map_manager = MapManager(
            self, self.params, 
            self.get_stable_batches_client,
            self.get_stable_batches_client_web
        )
        
        # Path service isolated in its own callback group.
        self.path_service = self.create_service(
            ComputePath, '/compute_path', self.handle_compute_path,
            callback_group=self.path_callback_group)

        self.put_down_service = self.create_service(
            PutDown, '/put_down', self.handle_put_down,
            callback_group=self.misc_callback_group)
        
        self.batch_color_service = self.create_service(
            GetBatchColorPattern, '/get_batch_color_pattern', self.handle_get_batch_color_pattern,
            callback_group=self.misc_callback_group)
        
        self.delete_batch_service = self.create_service(
            DeleteBatch, '/delete_batch', self.handle_delete_batch,
            callback_group=self.misc_callback_group)

        # Active batches query service (used by DecisionMaker via web_handler)
        self.get_active_batches_service = self.create_service(
            GetActiveBatches, '/pathfinder/get_active_batches', self.handle_get_active_batches,
            callback_group=self.misc_callback_group)

        # Backward-compatible alias
        self.get_active_batches_service_alias = self.create_service(
            GetActiveBatches, '/get_active_batches', self.handle_get_active_batches,
            callback_group=self.misc_callback_group)

        # Set team color used for timed 85s PAMI wall symmetry behavior.
        self.set_team_color_service = self.create_service(
            SetTeamColor, '/set_team_color', self.map_manager.handle_set_team_color,
            callback_group=self.map_callback_group)

        # Topic publisher for active batches (same payload fields as GetActiveBatches response)
        self.active_batches_pub = self.create_publisher(ActiveBatches, '/active_obstacles', 10)
        self.active_batches_timer = self.create_timer(1.0, self.publish_active_batches_topic)
        
        # Delegate update_map service to MapManager
        self.update_map_service = self.create_service(
            Trigger, '/update_map', self.map_manager.handle_update_map,
            callback_group=self.map_callback_group)

        # Start game flow: begin periodic update_map only after baseline lock completes
        self.start_game_service = self.create_service(
            Trigger, '/start_game', self.map_manager.handle_start_game,
            callback_group=self.map_callback_group)
        
        # Delegate clear_map service to MapManager
        self.clear_map_service = self.create_service(
            Trigger, '/clear_map', self.map_manager.handle_clear_map,
            callback_group=self.map_callback_group)
        
        # Delegate reset_map service to MapManager (same as update_map but with 'rrr' types_filter)
        self.reset_map_service = self.create_service(
            Trigger, '/reset_map', self.map_manager.handle_reset_map,
            callback_group=self.map_callback_group)
        
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
            effective_length,
            self.params['grid']['resolution_mm']
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
    
    def log_once(self, message: str, level: str = 'info'):
        """Log a message only if it's different from recent messages (prevents spam)"""
        # Check if this exact message was logged recently
        if message in self._last_log_messages:
            return  # Skip duplicate
        
        # Log the message
        if level == 'info':
            self.get_logger().info(message)
        elif level == 'warn':
            self.get_logger().warn(message)
        elif level == 'error':
            self.get_logger().error(message)
        elif level == 'debug':
            self.get_logger().debug(message)
        
        # Update history
        self._last_log_messages.append(message)
        if len(self._last_log_messages) > self._max_log_history:
            self._last_log_messages.pop(0)
    
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
                    obs.pose.x = float(obs_dict.get('x', 0.0))  # center X (mm)
                    obs.pose.y = float(obs_dict.get('y', 0.0))  # center Y (mm)
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
            pose_stamped.pose.orientation = MapManager.yaw_to_quaternion(theta)
            path_msg.poses.append(pose_stamped)
        
        return path_msg


    def publish_path_visualization(self, waypoints: List[Tuple[float, float, float]], 
                                   obstacles: List[dict],
                                   path_viz_pub):
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
        crates = [obs for obs in obstacles if obs['color'] == 'yellow' or obs['color'] == 'blue' or obs['color'] == 'red']
        
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
            
            # Position (on ground level) - x/y are centers
            crate_marker.pose.position.x = obs['x'] / 1000.0
            crate_marker.pose.position.y = obs['y'] / 1000.0
            crate_marker.pose.position.z = 0.03  # 30mm high (half of 150mm)
            crate_marker.pose.orientation = MapManager.yaw_to_quaternion(obs.get('theta', 0.0))
            
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
            
            marker_array.markers.append(crate_marker)
        
        path_viz_pub.publish(marker_array)
    
    def optimize_path_with_intersections(
    self,
    path: list,
    grid_resolution: float,
    max_added_distance_mm: float = 0.0,  # your threshold
    sample_step_mm: float = 10.0,  # Reduced from 50mm for better collision detection
    enable_direct_simplification: bool = False,
) -> list:
        """
        Simplify a path by replacing 4-point zig-zag segments with their line
        intersection when it is collision-free and does not add too much distance.

        Input path format:
            [(x, y, theta), ...]

        Returns:
            Optimized path [(x, y, theta), ...]
        """

        if len(path) < 3:
            return path

        def collision_check_segment(pose1, pose2):
            """
            Check collision along a straight segment by sampling points with interpolated orientation
            pose1, pose2: tuples of (x, y, theta)
            """
            x1, y1, theta1 = pose1
            x2, y2, theta2 = pose2
            dist = math.hypot(x2 - x1, y2 - y1)
            steps = max(10, int(dist / sample_step_mm))  # Increased minimum from 5 to 10
            for i in range(steps + 1):
                t = i / steps
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                theta = theta1 + t * (theta2 - theta1)  # Interpolate orientation
                if self.pathfinder.is_collision(x, y, theta, grid_resolution):
                    return True
            return False

        def first_collision_on_segment(pose1, pose2):
            """Return first collision sample (x, y, theta, i, steps) or None."""
            x1, y1, theta1 = pose1
            x2, y2, theta2 = pose2
            dist = math.hypot(x2 - x1, y2 - y1)
            steps = max(10, int(dist / sample_step_mm))
            for i in range(steps + 1):
                t = i / steps
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                theta = theta1 + t * (theta2 - theta1)
                if self.pathfinder.is_collision(x, y, theta, grid_resolution):
                    return (x, y, theta, i, steps)
            return None

        def segment_distance(p1, p2):
            return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

        def normalize_angle(theta):
            while theta > math.pi:
                theta -= 2 * math.pi
            while theta < -math.pi:
                theta += 2 * math.pi
            return theta

        def heading_candidates(theta):
            primary = normalize_angle(theta)
            reverse = normalize_angle(theta + math.pi)
            return [primary, reverse]

        def rotation_safe_at_point(x, y, theta_from, theta_to):
            """Check if in-place rotation at one point is collision-free."""
            angle_diff = theta_to - theta_from
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            checks = max(8, int(abs(angle_diff) / math.radians(5)))
            for i in range(checks + 1):
                ratio = i / checks
                check_theta = theta_from + ratio * angle_diff
                if self.pathfinder.is_collision(x, y, check_theta, grid_resolution):
                    return False
            return True

        def first_collision_on_rotation(x, y, theta_from, theta_to):
            """Return first colliding angle (theta, i, checks) or None."""
            angle_diff = theta_to - theta_from
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            checks = max(8, int(abs(angle_diff) / math.radians(5)))
            for i in range(checks + 1):
                ratio = i / checks
                check_theta = theta_from + ratio * angle_diff
                if self.pathfinder.is_collision(x, y, check_theta, grid_resolution):
                    return (check_theta, i, checks)
            return None

        def project_line_to_valid_point(x_start: float, y_start: float, direction_x: float, direction_y: float, theta: float, max_distance_mm: float = 1500.0, step_mm: float = 25.0):
            """
            Project a line from (x_start, y_start) in direction (direction_x, direction_y) 
            until finding a collision-free waypoint. Returns (x, y, distance) or None.
            """
            dist_mag = math.sqrt(direction_x**2 + direction_y**2)
            if dist_mag < 1e-6:
                return None
            
            dir_x_norm = direction_x / dist_mag
            dir_y_norm = direction_y / dist_mag
            
            # Walk along the line in small steps
            num_steps = int(max_distance_mm / step_mm)
            for step in range(1, num_steps + 1):
                test_dist = step * step_mm
                test_x = x_start + dir_x_norm * test_dist
                test_y = y_start + dir_y_norm * test_dist
                
                # Check collision at this point
                if not self.pathfinder.is_collision(test_x, test_y, theta, grid_resolution):
                    return (test_x, test_y, test_dist)
            
            return None

        optimized = path.copy()
        max_rounds = 6
        total_removed_intersection = 0
        total_removed_direct = 0
        rounds_used = 0
        rejection_logs_left = 30

        def _log_rejection(prefix: str, details: str):
            nonlocal rejection_logs_left
            if rejection_logs_left <= 0:
                return
            self.get_logger().info(f"🚫 simplify reject [{prefix}]: {details}")
            rejection_logs_left -= 1

        def _collision_debug_details(x: float, y: float, theta: float) -> str:
            """Return compact collision debug info for a pose."""
            try:
                hits = self.pathfinder.get_collision_debug_info(
                    x, y, theta, grid_resolution, max_items=2
                )
                if not hits:
                    return ""
                return f" | debug={'; '.join(str(h) for h in hits)}"
            except Exception:
                return ""

        for round_idx in range(max_rounds):
            rounds_used = round_idx + 1

            # First pass: intersection-based simplification
            removed = 0
            index = 0
            while index <= len(optimized) - 4:
                x1, y1, t1 = optimized[index]
                x2, y2, t2 = optimized[index + 1]
                x3, y3, t3 = optimized[index + 2]
                x4, y4, t4 = optimized[index + 3]

                # Compute intersection of (p1,p2) with (p3,p4)
                den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
                if abs(den) < 1e-6:
                    _log_rejection(
                        f"I r{round_idx+1} i{index}",
                        "segments nearly parallel (no stable intersection)"
                    )
                    index += 1
                    continue

                px = ((x1 * y2 - y1 * x2) * (x3 - x4) -
                    (x1 - x2) * (x3 * y4 - y3 * x4)) / den
                py = ((x1 * y2 - y1 * x2) * (y3 - y4) -
                    (y1 - y2) * (x3 * y4 - y3 * x4)) / den

                first_theta = math.atan2(py - y1, px - x1)
                second_theta_base = math.atan2(y4 - py, x4 - px)

                # More permissive and consistent with A->C simplification:
                # translation is checked with fixed travel heading, while
                # rotations are validated separately at waypoints.
                first_seg_collision = first_collision_on_segment((x1, y1, first_theta), (px, py, first_theta))
                start_rotation_ok = rotation_safe_at_point(x1, y1, t1, first_theta)

                second_candidate_results = []
                chosen_second_theta = None
                chosen_second_seg_collision = None
                chosen_mid_rotation_ok = False
                chosen_end_rotation_ok = False

                for second_theta in heading_candidates(second_theta_base):
                    second_seg_collision = first_collision_on_segment((px, py, second_theta), (x4, y4, second_theta))
                    intersection_rotation_ok = rotation_safe_at_point(px, py, first_theta, second_theta)
                    end_rotation_ok = rotation_safe_at_point(x4, y4, second_theta, t4)

                    second_candidate_results.append({
                        'theta': second_theta,
                        'seg_collision': second_seg_collision,
                        'mid_ok': intersection_rotation_ok,
                        'end_ok': end_rotation_ok,
                    })

                    if (
                        first_seg_collision is None
                        and start_rotation_ok
                        and second_seg_collision is None
                        and intersection_rotation_ok
                        and end_rotation_ok
                    ):
                        chosen_second_theta = second_theta
                        chosen_second_seg_collision = second_seg_collision
                        chosen_mid_rotation_ok = intersection_rotation_ok
                        chosen_end_rotation_ok = end_rotation_ok
                        break

                if chosen_second_theta is None:
                    best_candidate = second_candidate_results[0]
                    chosen_second_theta = best_candidate['theta']
                    chosen_second_seg_collision = best_candidate['seg_collision']
                    chosen_mid_rotation_ok = best_candidate['mid_ok']
                    chosen_end_rotation_ok = best_candidate['end_ok']

                translation_collision = (first_seg_collision is not None) or (chosen_second_seg_collision is not None)

                old_dist = segment_distance((x1, y1), (x2, y2)) + segment_distance((x2, y2), (x3, y3)) + segment_distance((x3, y3), (x4, y4))
                new_dist = segment_distance((x1, y1), (px, py)) + segment_distance((px, py), (x4, y4))
                added_distance = new_dist - old_dist

                distance_ok = added_distance <= max_added_distance_mm

                if (
                    (not translation_collision)
                    and start_rotation_ok
                    and chosen_mid_rotation_ok
                    and chosen_end_rotation_ok
                    and distance_ok
                ):
                    optimized[index + 1] = (px, py, first_theta)
                    del optimized[index + 2]
                    removed += 1
                    continue

                # Fallback: single-projection simplification
                # Try p1->proj1->p4 and p1->proj2->p4, keep shortest valid one.
                proj_old_dist = (
                    segment_distance((x1, y1), (x2, y2))
                    + segment_distance((x2, y2), (x3, y3))
                    + segment_distance((x3, y3), (x4, y4))
                )

                candidate_paths = []
                proj_reasons = []
                map_width_mm = float(self.params['map']['width_mm'])
                map_height_mm = float(self.params['map']['height_mm'])

                def _scan_projection_candidate(label: str, x_start: float, y_start: float, dir_x: float, dir_y: float):
                    mag = math.hypot(dir_x, dir_y)
                    if mag < 1e-6:
                        return None, f"{label} direction too small"

                    ux = dir_x / mag
                    uy = dir_y / mag
                    step_mm = 25.0
                    max_distance_mm = 1500.0
                    num_steps = int(max_distance_mm / step_mm)
                    last_reason = f"{label} no valid point within {max_distance_mm:.0f}mm"

                    for step in range(1, num_steps + 1):
                        travel_mm = step * step_mm
                        px_cand = x_start + ux * travel_mm
                        py_cand = y_start + uy * travel_mm

                        if not (0.0 <= px_cand <= map_width_mm and 0.0 <= py_cand <= map_height_mm):
                            last_reason = (
                                f"{label} out of map at ({px_cand:.1f},{py_cand:.1f}) "
                                f"bounds=(0..{map_width_mm:.0f},0..{map_height_mm:.0f})"
                            )
                            break

                        theta1 = math.atan2(py_cand - y1, px_cand - x1)
                        theta2 = math.atan2(y4 - py_cand, x4 - px_cand)

                        start_ok = rotation_safe_at_point(x1, y1, t1, theta1)
                        if not start_ok:
                            last_reason = f"{label} start rotation blocked"
                            continue

                        seg1_col = first_collision_on_segment((x1, y1, theta1), (px_cand, py_cand, theta1))
                        if seg1_col is not None:
                            last_reason = f"{label} seg1 collision at ({seg1_col[0]:.1f},{seg1_col[1]:.1f})"
                            continue

                        seg2_col = first_collision_on_segment((px_cand, py_cand, theta2), (x4, y4, theta2))
                        if seg2_col is not None:
                            last_reason = f"{label} seg2 collision at ({seg2_col[0]:.1f},{seg2_col[1]:.1f})"
                            continue

                        end_ok = rotation_safe_at_point(x4, y4, theta2, t4)
                        if not end_ok:
                            last_reason = f"{label} end rotation blocked"
                            continue

                        new_dist = segment_distance((x1, y1), (px_cand, py_cand)) + segment_distance((px_cand, py_cand), (x4, y4))
                        added_dist = new_dist - proj_old_dist
                        if added_dist > max_added_distance_mm:
                            last_reason = f"{label} added_dist={added_dist:.0f}mm > limit"
                            continue

                        return {
                            'name': label,
                            'x': px_cand,
                            'y': py_cand,
                            'theta': theta1,
                            'travel_mm': travel_mm,
                            'new_dist': new_dist,
                            'added_dist': added_dist,
                        }, None

                    return None, last_reason

                cand1, reason1 = _scan_projection_candidate("proj1", x1, y1, x2 - x1, y2 - y1)
                cand2, reason2 = _scan_projection_candidate("proj2", x4, y4, x4 - x3, y4 - y3)

                if cand1 is not None:
                    candidate_paths.append(cand1)
                elif reason1:
                    proj_reasons.append(reason1)

                if cand2 is not None:
                    candidate_paths.append(cand2)
                elif reason2:
                    proj_reasons.append(reason2)

                if candidate_paths:
                    best = min(candidate_paths, key=lambda c: c['new_dist'])
                    optimized[index + 1] = (best['x'], best['y'], best['theta'])
                    del optimized[index + 2]
                    removed += 1
                    self.get_logger().info(
                        f"✨ projection fallback [I r{round_idx+1} i{index}]: "
                        f"used={best['name']} proj@{best['travel_mm']:.0f}mm "
                        f"added={best['added_dist']:.0f}mm"
                    )
                    continue
                else:
                    _log_rejection(
                        f"I-proj r{round_idx+1} i{index}",
                        "; ".join(proj_reasons) if proj_reasons else "no valid projection path"
                    )

                # Log only the primary (first) rejection reason
                if first_seg_collision is not None:
                    ctheta, ci, csteps = first_seg_collision[2], first_seg_collision[3], first_seg_collision[4]
                    extra = _collision_debug_details(first_seg_collision[0], first_seg_collision[1], ctheta)
                    _log_rejection(
                        f"I r{round_idx+1} i{index}",
                        (
                            f"seg1 collision at ({first_seg_collision[0]:.1f},{first_seg_collision[1]:.1f}) "
                            f"sample={ci}/{csteps} theta={math.degrees(ctheta):.1f}° "
                            f"seg=({x1:.1f},{y1:.1f})->({px:.1f},{py:.1f})"
                            f"{extra}"
                        )
                    )
                elif not start_rotation_ok:
                    rot_hit = first_collision_on_rotation(x1, y1, t1, first_theta)
                    if rot_hit is not None:
                        rtheta, ri, rchecks = rot_hit
                        extra = _collision_debug_details(x1, y1, rtheta)
                        details = (
                            f"start rotation blocked at ({x1:.1f},{y1:.1f}) "
                            f"theta={math.degrees(rtheta):.1f}° sample={ri}/{rchecks} "
                            f"from={math.degrees(t1):.1f}° to={math.degrees(first_theta):.1f}°"
                            f"{extra}"
                        )
                    else:
                        details = "start rotation blocked"
                    _log_rejection(
                        f"I r{round_idx+1} i{index}",
                        details
                    )
                elif chosen_second_seg_collision is not None:
                    ctheta, ci, csteps = chosen_second_seg_collision[2], chosen_second_seg_collision[3], chosen_second_seg_collision[4]
                    extra = _collision_debug_details(chosen_second_seg_collision[0], chosen_second_seg_collision[1], ctheta)
                    _log_rejection(
                        f"I r{round_idx+1} i{index}",
                        (
                            f"seg2 collision at ({chosen_second_seg_collision[0]:.1f},{chosen_second_seg_collision[1]:.1f}) "
                            f"sample={ci}/{csteps} theta={math.degrees(ctheta):.1f}° "
                            f"seg=({px:.1f},{py:.1f})->({x4:.1f},{y4:.1f})"
                            f"{extra}"
                        )
                    )
                elif not chosen_mid_rotation_ok:
                    rot_hit = first_collision_on_rotation(px, py, first_theta, chosen_second_theta)
                    if rot_hit is not None:
                        rtheta, ri, rchecks = rot_hit
                        extra = _collision_debug_details(px, py, rtheta)
                        details = (
                            f"intersection rotation blocked at ({px:.1f},{py:.1f}) "
                            f"theta={math.degrees(rtheta):.1f}° sample={ri}/{rchecks} "
                            f"from={math.degrees(first_theta):.1f}° to={math.degrees(chosen_second_theta):.1f}°"
                            f"{extra}"
                        )
                    else:
                        details = "intersection rotation blocked"
                    _log_rejection(
                        f"I r{round_idx+1} i{index}",
                        details
                    )
                elif not chosen_end_rotation_ok:
                    rot_hit = first_collision_on_rotation(x4, y4, chosen_second_theta, t4)
                    if rot_hit is not None:
                        rtheta, ri, rchecks = rot_hit
                        extra = _collision_debug_details(x4, y4, rtheta)
                        details = (
                            f"end rotation blocked at ({x4:.1f},{y4:.1f}) "
                            f"theta={math.degrees(rtheta):.1f}° sample={ri}/{rchecks} "
                            f"from={math.degrees(chosen_second_theta):.1f}° to={math.degrees(t4):.1f}°"
                            f"{extra}"
                        )
                    else:
                        details = "end rotation blocked"
                    _log_rejection(
                        f"I r{round_idx+1} i{index}",
                        details
                    )
                elif not distance_ok:
                    _log_rejection(
                        f"I r{round_idx+1} i{index}",
                        (
                            f"added_distance={added_distance:.1f}mm > limit={max_added_distance_mm:.1f}mm "
                            f"old={old_dist:.1f}mm new={new_dist:.1f}mm"
                        )
                    )

                index += 1

            # Second pass (optional): simplify 3-point bends (A->B->C => A->C)
            triple_removed = 0
            if enable_direct_simplification:
                index = 0
                while index <= len(optimized) - 3:
                    x1, y1, t1 = optimized[index]
                    x2, y2, _ = optimized[index + 1]
                    x3, y3, t3 = optimized[index + 2]

                    direct_theta_base = math.atan2(y3 - y1, x3 - x1)
                    chosen_direct_theta = None
                    direct_seg_collision = None
                    end_rotation_ok = False

                    for direct_theta in heading_candidates(direct_theta_base):
                        candidate_seg_collision = first_collision_on_segment(
                            (x1, y1, direct_theta),
                            (x3, y3, direct_theta)
                        )
                        candidate_end_rotation_ok = rotation_safe_at_point(x3, y3, direct_theta, t3)

                        if candidate_seg_collision is None and candidate_end_rotation_ok:
                            chosen_direct_theta = direct_theta
                            direct_seg_collision = candidate_seg_collision
                            end_rotation_ok = candidate_end_rotation_ok
                            break

                        if chosen_direct_theta is None:
                            chosen_direct_theta = direct_theta
                            direct_seg_collision = candidate_seg_collision
                            end_rotation_ok = candidate_end_rotation_ok

                    translation_collision = direct_seg_collision is not None

                    old_dist = segment_distance((x1, y1), (x2, y2)) + segment_distance((x2, y2), (x3, y3))
                    new_dist = segment_distance((x1, y1), (x3, y3))
                    added_distance = new_dist - old_dist

                    distance_ok = added_distance <= max_added_distance_mm

                    if (
                        (not translation_collision)
                        and end_rotation_ok
                        and distance_ok
                    ):
                        del optimized[index + 1]
                        triple_removed += 1
                        continue

                    # Log only the primary (first) rejection reason
                    if translation_collision and direct_seg_collision is not None:
                        ctheta, ci, csteps = direct_seg_collision[2], direct_seg_collision[3], direct_seg_collision[4]
                        extra = _collision_debug_details(direct_seg_collision[0], direct_seg_collision[1], ctheta)
                        _log_rejection(
                            f"D r{round_idx+1} i{index}",
                            (
                                f"direct segment collision at ({direct_seg_collision[0]:.1f},{direct_seg_collision[1]:.1f}) "
                                f"sample={ci}/{csteps} theta={math.degrees(ctheta):.1f}° "
                                f"seg=({x1:.1f},{y1:.1f})->({x3:.1f},{y3:.1f})"
                                f"{extra}"
                            )
                        )
                    elif not end_rotation_ok:
                        rot_hit = first_collision_on_rotation(x3, y3, chosen_direct_theta, t3)
                        if rot_hit is not None:
                            rtheta, ri, rchecks = rot_hit
                            extra = _collision_debug_details(x3, y3, rtheta)
                            details = (
                                f"end rotation blocked at ({x3:.1f},{y3:.1f}) "
                                f"theta={math.degrees(rtheta):.1f}° sample={ri}/{rchecks} "
                                f"from={math.degrees(chosen_direct_theta):.1f}° to={math.degrees(t3):.1f}°"
                                f"{extra}"
                            )
                        else:
                            details = "end rotation blocked"
                        _log_rejection(
                            f"D r{round_idx+1} i{index}",
                            details
                        )
                    elif not distance_ok:
                        _log_rejection(
                            f"D r{round_idx+1} i{index}",
                            (
                                f"added_distance={added_distance:.1f}mm > limit={max_added_distance_mm:.1f}mm "
                                f"old={old_dist:.1f}mm new={new_dist:.1f}mm"
                            )
                        )

                    index += 1

            total_removed_intersection += removed
            total_removed_direct += triple_removed

            if (removed + triple_removed) == 0:
                break

        total_removed = total_removed_intersection + total_removed_direct
        if total_removed > 0:
            self.get_logger().info(
                f"✨ Path simplification removed {total_removed} waypoints in {rounds_used} round(s) "
                f"(intersection={total_removed_intersection}, direct={total_removed_direct})"
            )

        return optimized
    
    def _is_goal_on_batch(self, goal_x: float, goal_y: float, obstacles: List[dict]) -> bool:
        """
        Check if goal position overlaps with any batch obstacle.
        This is used to automatically enable batch pickup mode.
        
        Args:
            goal_x, goal_y: Goal position in mm
            obstacles: List of obstacle dictionaries
            
        Returns:
            True if goal overlaps with a batch (FX or SX), False otherwise
        """
        # Enhanced margin to account for robot footprint when picking up batch
        # Robot is ~370mm wide, so we need more margin to detect nearby goals
        margin = 200.0  # mm - increased from 50mm to catch goals near batches
        
        batch_obstacles = [obs for obs in obstacles if obs.get('id', '').startswith(('fx_', 'sx_'))]
        
        if not batch_obstacles:
            return False
        
        # Check if goal center is inside any batch obstacle (x/y are centers)
        for obs in batch_obstacles:
            obs_id = obs.get('id', '')
            obs_x = obs['x']  # center X
            obs_y = obs['y']  # center Y
            obs_width = obs['width']
            obs_height = obs['height']
            
            # x/y are already centers
            batch_center_x = obs_x
            batch_center_y = obs_y
            distance = math.sqrt((goal_x - batch_center_x)**2 + (goal_y - batch_center_y)**2)
            
            # Check if goal is inside obstacle bounds (with margin)
            in_x_range = obs_x - obs_width / 2.0 - margin <= goal_x <= obs_x + obs_width / 2.0 + margin
            in_y_range = obs_y - obs_height / 2.0 - margin <= goal_y <= obs_y + obs_height / 2.0 + margin
            
            if in_x_range and in_y_range:
                self.get_logger().info(
                    f"  ✅ Goal ({goal_x:.1f}, {goal_y:.1f}) overlaps with batch '{obs_id}' "
                    f"(distance={distance:.1f}mm from center)"
                )
                return True
        
        self.get_logger().warn(
            f"  ⚠️ Goal ({goal_x:.1f}, {goal_y:.1f}) does NOT overlap with any batch "
            f"(checked {len(batch_obstacles)} batches with {margin}mm margin)"
        )
        return False
    
    def handle_compute_path(self, request: ComputePath.Request, 
                           response: ComputePath.Response) -> ComputePath.Response:
        """
        Service callback: compute path from start to goal
        Uses the same advanced pathfinding logic as the local test script.
        """
        start_time = time.time()
        response.can_rotate = True
        
        # Extract start and goal (convert m to mm)
        start_x = request.start_pose.pose.position.x * 1000.0
        start_y = request.start_pose.pose.position.y * 1000.0
        start_theta = MapManager.quaternion_to_yaw(request.start_pose.pose.orientation)
        self.last_robot_pose_mm = (start_x, start_y, start_theta)
        
        goal_x = request.goal_pose.pose.position.x * 1000.0
        goal_y = request.goal_pose.pose.position.y * 1000.0
        goal_theta = MapManager.quaternion_to_yaw(request.goal_pose.pose.orientation)
        
        self.get_logger().info('='*60)
        self.get_logger().info(f"🔍 Computing path:")
        self.get_logger().info(f"   Start: ({start_x:.0f}, {start_y:.0f}) θ={math.degrees(start_theta):.1f}° (rad={start_theta:.3f})")
        self.get_logger().info(f"   Goal:  ({goal_x:.0f}, {goal_y:.0f}) θ={math.degrees(goal_theta):.1f}° (rad={goal_theta:.3f})")
        
        # Filter obstacles by color
        active_obstacles = self.map_manager.get_active_obstacles()

        # Optionally ignore selected batch obstacles for planning while keeping metadata available
        # Request carries known_ids (e.g., ['S6_WEB']) and we skip obstacles with ids like:
        #   fx_<known_id>_memberN / sx_<known_id>_memberN
        untracked_batches = [kid.strip() for kid in list(request.untracked_batches) if kid.strip()]
        if untracked_batches:
            before_count = len(active_obstacles)
            active_obstacles = [
                obs for obs in active_obstacles
                if not any(
                    f"fx_{known_id}_" in obs.get('id', '') or f"sx_{known_id}_" in obs.get('id', '')
                    for known_id in untracked_batches
                )
            ]
            removed_count = before_count - len(active_obstacles)
            self.get_logger().info(
                f"   Untracked batches for planning: {untracked_batches} (ignored {removed_count} obstacles)"
            )
        
        self.get_logger().info(f"   Dynamic obstacles: {len(active_obstacles)}")
        self.get_logger().info(f"   Permanent area: (0,600)→(450,2400) always blocked")
        
        # Log obstacle details for debugging
        batch_count = sum(1 for obs in active_obstacles if obs.get('id', '').startswith(('fx_', 'sx_')))
        self.get_logger().info(f"   Batch obstacles: {batch_count}")
        
        try:
            # Set obstacles in pathfinder
            grid_resolution = self.params['grid']['resolution_mm']
            
            # All obstacles are dynamic (permanent area is handled automatically)
            self.pathfinder.set_obstacles(active_obstacles, grid_resolution, static_walls=None)
            
            # Automatically detect if goal is targeting a batch for pickup
            allow_goal_collision = self._is_goal_on_batch(goal_x, goal_y, active_obstacles)
            if allow_goal_collision:
                self.get_logger().info("🎯 Goal overlaps with batch obstacle - enabling batch pickup mode")
            
            
            # Run Hybrid A* algorithm (same as local script)
            path = self.pathfinder.find_path(
                start_pose=(start_x, start_y, start_theta),
                goal_pose=(goal_x, goal_y, goal_theta),
                grid_resolution=grid_resolution,
                max_iterations=self.params['pathfinding']['max_iterations'],
                allow_goal_collision=allow_goal_collision,
                time_limit_sec=5.0,
            )

            # can_rotate is enabled by default.
            # If start escape fallback was required, report can_rotate=False.
            used_start_fallback = bool(getattr(self.pathfinder, 'last_used_start_fallback', False))
            start_escape_mode = str(getattr(self.pathfinder, 'last_start_escape_mode', 'unknown'))
            used_bridge_fallback = bool(getattr(self.pathfinder, 'last_used_bridge_fallback', False))
            bridge_point = getattr(self.pathfinder, 'last_bridge_point', None)
            if used_start_fallback:
                response.can_rotate = False
            
            if path:
                self.get_logger().info(f"✅ Found path: {len(path)} waypoints")
                if used_bridge_fallback and bridge_point is not None:
                    bx, by, _ = bridge_point
                    self.get_logger().info(
                        f"🧭 Bridge fallback used (y>=1500mm) via ({bx:.1f}, {by:.1f})"
                    )
                # Smooth path (always enabled, same as local script)
                smoothed = self.pathfinder.smooth_path(path, grid_resolution)
                reduction = 100 * (1 - len(smoothed) / len(path)) if len(path) > 0 else 0
                self.get_logger().info(f"🔄 Smoothed to {len(smoothed)} waypoints ({reduction:.1f}% reduction)")
                
                # Ensure final waypoint is the exact requested goal pose.
                if len(smoothed) > 0:
                    smoothed[-1] = (goal_x, goal_y, goal_theta)
                
                # Validate that each waypoint can safely rotate to travel angle
                # Validate that each waypoint can safely rotate to travel angle
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
                
                # FINAL VALIDATION: Check each waypoint can safely rotate to travel toward next waypoint
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
                smoothed = self.optimize_path_with_intersections(
                    smoothed, 
                    grid_resolution=grid_resolution,
                    max_added_distance_mm=1000.0,  # Allow up to 1000mm extra distance for straighter paths
                    enable_direct_simplification=True,
                )

                # Post-optimization cleanup: remove waypoints that can be skipped
                # without translation (rotation-only points created/left after simplification).
                if len(smoothed) >= 3:
                    def _rotation_safe_here(x, y, theta_from, theta_to):
                        d = theta_to - theta_from
                        while d > math.pi:
                            d -= 2 * math.pi
                        while d < -math.pi:
                            d += 2 * math.pi
                        checks = max(8, int(abs(d) / math.radians(5)))
                        for s in range(checks + 1):
                            r = s / checks
                            th = theta_from + r * d
                            if self.pathfinder.is_collision(x, y, th, grid_resolution):
                                return False
                        return True

                    def _segment_clear_constant_theta(x1, y1, x2, y2, theta):
                        dist = math.hypot(x2 - x1, y2 - y1)
                        steps = max(10, int(dist / 10.0))
                        for s in range(steps + 1):
                            r = s / steps
                            sx = x1 + r * (x2 - x1)
                            sy = y1 + r * (y2 - y1)
                            if self.pathfinder.is_collision(sx, sy, theta, grid_resolution):
                                return False
                        return True

                    compact = [smoothed[0]]
                    skipped_no_translation = 0
                    i = 1
                    no_translation_dist_mm = 60.0

                    while i < len(smoothed) - 1:
                        prev_x, prev_y, prev_theta = compact[-1]
                        curr_x, curr_y, _ = smoothed[i]
                        next_x, next_y, _ = smoothed[i + 1]

                        prev_to_curr = math.hypot(curr_x - prev_x, curr_y - prev_y)

                        if prev_to_curr <= no_translation_dist_mm:
                            direct_theta = math.atan2(next_y - prev_y, next_x - prev_x)

                            can_skip = (
                                _rotation_safe_here(prev_x, prev_y, prev_theta, direct_theta)
                                and _segment_clear_constant_theta(prev_x, prev_y, next_x, next_y, direct_theta)
                            )

                            if can_skip:
                                skipped_no_translation += 1
                                i += 1
                                continue

                        compact.append(smoothed[i])
                        i += 1

                    compact.append(smoothed[-1])

                    if skipped_no_translation > 0:
                        smoothed = compact
                        self.get_logger().info(
                            f"🧹 Removed {skipped_no_translation} no-translation waypoint(s) after simplification"
                        )

                # Final heading cleanup: align waypoint orientation to outgoing segment when safe.
                if len(smoothed) >= 2:
                    aligned = list(smoothed)

                    def _rotation_safe_at_waypoint(x, y, theta_from, theta_to):
                        d = theta_to - theta_from
                        while d > math.pi:
                            d -= 2 * math.pi
                        while d < -math.pi:
                            d += 2 * math.pi
                        checks = max(8, int(abs(d) / math.radians(5)))
                        for s in range(checks + 1):
                            r = s / checks
                            th = theta_from + r * d
                            if self.pathfinder.is_collision(x, y, th, grid_resolution):
                                return False
                        return True

                    for i in range(len(aligned) - 1):
                        x, y, theta = aligned[i]
                        nx, ny, _ = aligned[i + 1]
                        desired_theta = math.atan2(ny - y, nx - x)

                        if (
                            not self.pathfinder.is_collision(x, y, desired_theta, grid_resolution)
                            and _rotation_safe_at_waypoint(x, y, theta, desired_theta)
                        ):
                            aligned[i] = (x, y, desired_theta)

                    smoothed = aligned

                if used_start_fallback and len(smoothed) > 0:
                    # Keep the robot's requested start pose as waypoint[0] in the
                    # returned path when start escape fallback was required.
                    first_x, first_y, first_theta = smoothed[0]
                    same_start = (
                        abs(first_x - start_x) < 1e-6
                        and abs(first_y - start_y) < 1e-6
                        and abs(first_theta - start_theta) < 1e-9
                    )
                    if not same_start:
                        smoothed = [(start_x, start_y, start_theta)] + smoothed
                        self.get_logger().info(
                            f"↪ Start escape fallback ({start_escape_mode}) used: prepended original start pose as first waypoint"
                        )

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
                    self.get_logger().info(
                        f"   {i+1:3d}. ({x:7.1f}, {y:7.1f}) θ={math.degrees(theta):6.1f}°"
                    )
                
                # Convert to ROS2 Path message
                response.path = self.convert_to_path_msg(smoothed, request.start_pose.header)
                response.success = True
                response.num_waypoints = len(smoothed)
                response.path_length_m = total_dist / 1000.0
                response.message = f"Path found: {len(smoothed)} waypoints, {response.path_length_m:.2f}m"
                response.message += f" (can_rotate={str(response.can_rotate).lower()})"
                
                # Add STM32 go_to commands to response
                response.stm32_commands = self.waypoints_to_stm32_go_to_commands(smoothed, speed=100)
                
                # Publish visualization
                self.publish_path_visualization(smoothed, active_obstacles, self.map_manager.path_viz_pub)
                
                self.get_logger().info(f"✅ Success!")
            else:
                response.success = False
                response.message = "No path found - start/goal in collision or blocked"
                response.message += f" (can_rotate={str(response.can_rotate).lower()})"
                self.get_logger().warn(f"❌ {response.message}")
                start_collision = self.pathfinder.is_collision(start_x, start_y, start_theta, grid_resolution)
                goal_collision = self.pathfinder.is_collision(goal_x, goal_y, goal_theta, grid_resolution)

                self.get_logger().warn(
                    "   Failure diagnostics: "
                    f"start_collision={start_collision}, "
                    f"goal_collision={goal_collision}, "
                    f"allow_goal_collision={allow_goal_collision}"
                )

                if start_collision:
                    self.get_logger().warn(
                        f"   Start pose in collision: ({start_x:.1f}, {start_y:.1f}, {math.degrees(start_theta):.1f}°)"
                    )
                    start_hits = self.pathfinder.get_collision_debug_info(
                        start_x, start_y, start_theta, grid_resolution, max_items=8
                    )
                    for idx, hit in enumerate(start_hits, start=1):
                        self.get_logger().warn(f"      start_collision[{idx}]: {hit}")

                if goal_collision and not allow_goal_collision:
                    self.get_logger().warn(
                        f"   Goal pose in collision: ({goal_x:.1f}, {goal_y:.1f}, {math.degrees(goal_theta):.1f}°)"
                    )
                    goal_hits = self.pathfinder.get_collision_debug_info(
                        goal_x, goal_y, goal_theta, grid_resolution, max_items=8
                    )
                    for idx, hit in enumerate(goal_hits, start=1):
                        self.get_logger().warn(f"      goal_collision[{idx}]: {hit}")

                if goal_collision and allow_goal_collision:
                    self.get_logger().warn(
                        "   Goal is in collision but overlap is allowed for this request (goal on batch)."
                    )

                if (not start_collision) and ((not goal_collision) or allow_goal_collision):
                    self.get_logger().warn(
                        "   Start/goal are collision-free under current rules; failure is likely blocked connectivity "
                        "(obstacle layout, corridor width, or search limits)."
                    )

                self.get_logger().warn("Possible issues:")
                self.get_logger().warn("  - Start or goal in collision with obstacles")
                self.get_logger().warn("  - Path blocked by obstacles")
                self.get_logger().warn("  - Try adjusting start/goal positions")
        
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            response.can_rotate = True
            self.get_logger().error(f"❌ Planning failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        finally:
            response.planning_time_ms = (time.time() - start_time) * 1000.0
            response.num_obstacles_used = len(active_obstacles)
            self.get_logger().info(f"   Planning time: {response.planning_time_ms:.1f}ms")
            self.get_logger().info('='*60)
        return response

    def _get_robot_pose_for_put_down(self) -> Optional[Tuple[float, float, float]]:
        """Get robot pose (x_mm, y_mm, theta_rad) from TF, fallback to last start pose."""
        base_frames = ['base_link', 'base_footprint']

        for base_frame in base_frames:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    'map',
                    base_frame,
                    rclpy.time.Time()
                )
                x_mm = tf_msg.transform.translation.x * 1000.0
                y_mm = tf_msg.transform.translation.y * 1000.0
                theta = MapManager.quaternion_to_yaw(tf_msg.transform.rotation)
                self.last_robot_pose_mm = (x_mm, y_mm, theta)
                return self.last_robot_pose_mm
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue
            except Exception:
                continue

        return self.last_robot_pose_mm

    def handle_put_down(self, request: PutDown.Request, response: PutDown.Response) -> PutDown.Response:
        """
        Add a placed 4-member batch as obstacles using robot pose and heading.

        Request:
          - direction: 1 (front) or 0 (back)
          - color: 'yellow' or 'blue'
                    - use_pose: if true, use (x_mm, y_mm, theta_deg) from request
        """
        direction = int(request.direction)
        if direction not in (0, 1):
            response.success = False
            response.message = "direction must be 0 (back) or 1 (front)"
            response.obstacle_ids = []
            return response

        color = request.color.strip().lower()
        if color not in ('yellow', 'blue'):
            response.success = False
            response.message = "color must be 'yellow' or 'blue'"
            response.obstacle_ids = []
            return response

        if bool(request.use_pose):
            robot_x_mm = float(request.x_mm)
            robot_y_mm = float(request.y_mm)
            robot_theta = math.radians(float(request.theta_deg))
            self.last_robot_pose_mm = (robot_x_mm, robot_y_mm, robot_theta)
        else:
            robot_pose = self._get_robot_pose_for_put_down()
            if robot_pose is None:
                response.success = False
                response.message = "Robot pose unavailable (no TF and no previous start pose)"
                response.obstacle_ids = []
                return response

            robot_x_mm, robot_y_mm, robot_theta = robot_pose

        # Geometry requested by user
        crate_length_mm = 150.0
        insert_offset_mm = float(getattr(request, 'insert_offset_mm', 0.0) or 0.0)

        # Forward offset: 150mm + 50mm, then center by +half crate length
        robot_to_batch_face_mm = 150.0 + 50.0 + insert_offset_mm
        center_forward_offset_mm = robot_to_batch_face_mm + (crate_length_mm / 2.0)
        if direction == 0:
            center_forward_offset_mm *= -1.0

        ux = math.cos(robot_theta)
        uy = math.sin(robot_theta)

        batch_center_x = robot_x_mm + center_forward_offset_mm * ux
        batch_center_y = robot_y_mm + center_forward_offset_mm * uy

        stamp = self.get_clock().now().to_msg()
        success, message, created_ids, _known_id = self.map_manager.register_put_down_batch(
            batch_center_x_mm=batch_center_x,
            batch_center_y_mm=batch_center_y,
            robot_x_mm=robot_x_mm,
            robot_y_mm=robot_y_mm,
            robot_theta=robot_theta,
            color=color,
            stamp=stamp,
        )

        response.success = success
        response.obstacle_ids = created_ids
        if success:
            response.message = (
                f"{message} ({'front' if direction == 1 else 'back'}) at robot pose "
                f"({robot_x_mm:.1f}, {robot_y_mm:.1f}, {math.degrees(robot_theta):.1f}deg)"
            )
            self.get_logger().info(f"✅ {response.message}")
        else:
            response.message = message
            self.get_logger().warning(f"⚠️ put_down failed: {message}")
        return response

    def handle_get_batch_color_pattern(self, request, response):
        """
        Service handler: Get color pattern for a batch by known_id or position
        If known_id is empty, finds closest batch to the provided position
        Returns the pattern exactly as saved in the batch.
        """

        known_id = request.batch_known_id.strip()

        batch = None
        
        # Get all batches from map manager
        all_batches = self.map_manager.get_all_batches()
        
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
                # Use log_once to avoid spamming same batch info
                self.log_once(f"batch known_id={b.known_id} and  its center : {b.center}")
                
                # Use batch center for distance calculation
                # For SX batches, center is set from predefined positions
                # For FX/GX batches, center is calculated from members
                batch_center_x = b.center.x
                batch_center_y = b.center.y
                
                # Calculate distance
                distance = math.sqrt((batch_center_x - pos_x)**2 + (batch_center_y - pos_y)**2)
                
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
            
            # Check if closest batch is within 0.33m (330mm) tolerance
            if min_distance > 0.33:
                response.success = False
                response.color_pattern = ""
                response.direction = ""
                response.batch_center.x = 0.0
                response.batch_center.y = 0.0
                response.batch_center.z = 0.0
                response.message = f"Closest batch is too far ({min_distance*1000:.0f}mm > 330mm limit)"
                self.get_logger().warn(
                    f"❌ Closest batch '{closest_batch.known_id}' is {min_distance*1000:.0f}mm away "
                    f"(exceeds 330mm limit) - skipping"
                )
                return response
            
            batch = closest_batch
            self.get_logger().info(
                f"📍 Found closest batch '{batch.known_id}' at distance {min_distance*1000:.0f}mm "
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

        # Use batch center (already set correctly for both SX and FX batches)
        response.batch_center.x = batch.center.x
        response.batch_center.y = batch.center.y
        response.batch_center.z = batch.center.z

        # Direction
        direction = batch.direction.lower()
        response.direction = direction

        # ✅ USE SAVED PATTERN DIRECTLY - No sorting, no reconstruction
        color_pattern = batch.pattern.lower() if batch.pattern else ""
        #color_pattern = color_pattern[::-1]
        # Validate pattern has content
        if not color_pattern:
            # Fallback: reconstruct from members only if pattern is empty
            self.get_logger().warn(f"⚠️ Batch '{batch.known_id}' has no saved pattern, reconstructing from members")
            
            # Copy members
            members = list(batch.members)

            # Sort members based on direction
            if direction == "vertical":
                # bottom → top, left → right
                members.sort(
                    key=lambda m: (m.map_pos.y, m.map_pos.x),
                    reverse=False
                )
            else:  # horizontal
                # left → right, bottom → top
                members.sort(
                    key=lambda m: (m.map_pos.x, m.map_pos.y),
                    reverse=False
                )

            # Build color pattern from sorted members
            for member in members:
                if member.color == "yellow":
                    color_pattern += "y"
                elif member.color == "blue":
                    color_pattern += "b"
                else:
                    color_pattern += "?"

        response.success = True
        response.color_pattern = color_pattern
        
        # Always return the actual batch known_id in the message
        response.message = batch.known_id
        
        # Log details
        if known_id == "":
            self.get_logger().info(
                f"Closest batch '{batch.known_id}' ({direction}): {color_pattern}"
            )
        else:
            self.get_logger().info(
                f"Batch '{known_id}' ({direction}): {color_pattern}"
            )

        self.get_logger().info(
            f"📋 Batch '{batch.known_id}' → pattern='{color_pattern}', "
            f"direction={direction}, members={len(batch.members)}, "
            f"center=({batch.center.x:.1f}, {batch.center.y:.1f})"
        )

        return response
    
    def handle_get_active_batches(self, request: GetActiveBatches.Request, response: GetActiveBatches.Response) -> GetActiveBatches.Response:
        """
        Service handler: Get information about all active batches
        Returns arrays of batch information (known_id, batch_type, member_count, direction, pattern, theta_degrees, center, quality)
        """
        try:
            # Get batch info from map manager
            batch_info = self.map_manager.get_active_batches_info()
            
            # Populate response arrays
            response.known_ids = batch_info['known_ids']
            response.batch_types = batch_info['batch_types']
            response.member_counts = batch_info['member_counts']
            response.directions = batch_info['directions']
            response.patterns = batch_info['patterns']
            response.theta_degrees = batch_info['theta_degrees']
            response.centers = batch_info['centers']
            response.qualities = batch_info['qualities']
            
            signature = (
                len(response.known_ids),
                response.batch_types.count('fx'),
                response.batch_types.count('sx'),
            )
            if signature != self._last_active_batches_signature:
                self.get_logger().info(
                    f"📊 Active batches query: {signature[0]} batches "
                    f"(FX: {signature[1]}, SX: {signature[2]})"
                )
                self._last_active_batches_signature = signature
            else:
                self.get_logger().debug(
                    f"📊 Active batches query unchanged: {signature[0]} batches "
                    f"(FX: {signature[1]}, SX: {signature[2]})"
                )

            # Mirror to topic so consumers can subscribe instead of polling service.
            self.publish_active_batches_topic(batch_info=batch_info)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error getting active batches: {e}")
            # Return empty arrays on error
            response.known_ids = []
            response.batch_types = []
            response.member_counts = []
            response.directions = []
            response.patterns = []
            response.theta_degrees = []
            response.centers = []
            response.qualities = []
        
        return response

    def publish_active_batches_topic(self, batch_info: Optional[Dict[str, list]] = None) -> None:
        """Publish active batches on /active_obstacles using ActiveBatches message."""
        try:
            if batch_info is None:
                batch_info = self.map_manager.get_active_batches_info()

            msg = ActiveBatches()
            msg.centers = batch_info.get('centers', [])
            msg.known_ids = batch_info.get('known_ids', [])
            msg.batch_types = batch_info.get('batch_types', [])
            msg.member_counts = batch_info.get('member_counts', [])
            msg.directions = batch_info.get('directions', [])
            msg.patterns = batch_info.get('patterns', [])
            msg.theta_degrees = batch_info.get('theta_degrees', [])
            msg.qualities = batch_info.get('qualities', [])
            self.active_batches_pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"Active batches topic publish skipped: {e}")
    
    def handle_delete_batch(self, request, response):
        """
        Service handler: Delete a batch by its known_id
        
        This removes the batch from both FX and SX storage,
        useful when a batch is picked up and should no longer be considered.
        """
        known_id = request.known_id.strip()
        
        if not known_id:
            response.success = False
            response.message = "Empty known_id provided"
            self.get_logger().warn("❌ Delete batch request with empty known_id")
            return response
        
        # Delegate to map_manager
        success = self.map_manager.delete_batch(known_id)
        
        if success:
            response.success = True
            response.message = f"Batch '{known_id}' deleted successfully"
            self.get_logger().info(f"✅ Batch '{known_id}' deleted")
        else:
            response.success = False
            response.message = f"Batch '{known_id}' not found"
            self.get_logger().warn(f"❌ Batch '{known_id}' not found for deletion")
        
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
    
    # Use MultiThreadedExecutor to handle multiple service requests simultaneously
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=8)  # 8 threads for parallel requests
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()