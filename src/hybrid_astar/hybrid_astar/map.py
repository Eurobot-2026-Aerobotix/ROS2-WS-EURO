#!/usr/bin/env python3
"""
Map Manager - Handles obstacle and batch management for path planning
Manages dynamic obstacles from ArUco mapper and enemy detection
"""

import math
from typing import Dict, List, Optional
from threading import Lock

import rclpy
from rclpy.node import Node

# ROS2 messages
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray

# Custom messages
from hybrid_astar.msg import Obstacle, ObstacleArray
from aruco_mapper_msgs.msg import ArucoMapperState, Batch
from aruco_mapper_msgs.srv import GetStableBatches


class MapManager:
    """
    Manages map-related data: obstacles, batches, and their visualization
    
    Responsibilities:
    - Maintain obstacle database (FX, SX, and enemy obstacles)
    - Process ArUco mapper state updates (via /update_map trigger)
    - Manage batch storage and color patterns
    - Handle obstacle clearing and relative obstacle updates
    - Provide obstacle visualization markers
    """
    
    def __init__(self, node: Node, params: dict, get_stable_batches_client, get_stable_batches_client_web=None):
        """
        Initialize the map manager
        
        Args:
            node: ROS2 node instance (for logging, time, etc)
            params: Parameter dictionary containing robot, map, and obstacle configs
            get_stable_batches_client: Service client for /aruco_mapper/get_stable_batches
            get_stable_batches_client_web: Optional service client for /aruco_mapper/get_stable_batches_web
        """
        self.node = node
        self.params = params
        self.get_stable_batches_client = get_stable_batches_client
        self.get_stable_batches_client_web = get_stable_batches_client_web
        
        # Predefined batch positions (in meters) for SX batches
        self.predefined_batch_positions = {
            "s1": {"x": 0.80, "y": 0.175, "phi_degrees": 0.0},
            "s2": {"x": 1.60, "y": 0.175, "phi_degrees": 0.0},
            "s3": {"x": 1.20, "y": 1.15, "phi_degrees": 90.0},
            "s4": {"x": 1.825, "y": 1.1, "phi_degrees": 90.0},
            "s5": {"x": 1.20, "y": 1.85, "phi_degrees": 90.0},
            "s6": {"x": 1.825, "y": 1.9, "phi_degrees": 90.0},
            "s7": {"x": 0.80, "y": 2.8, "phi_degrees": 90.0},
            "s8": {"x": 1.60, "y": 2.8, "phi_degrees": 90.0},
        }
        
        # Obstacle database (in-memory only) - tracking source
        self.obstacles: List[Obstacle] = []
        self.obstacles_lock = Lock()
        
        # Track obstacle sources: 'fx', 'sx', or 'enemy'
        self.obstacle_sources: Dict[str, str] = {}  # obstacle_id -> source
        
        # Batch storage (for color pattern queries) - separate for fx and sx
        # Using known_id as key to avoid overwrites when batch_id is reused
        self.batches_fx: Dict[str, Batch] = {}
        self.batches_sx: Dict[str, Batch] = {}
        self.batches_lock = Lock()
        
        # Static obstacles - using permanent area instead of JSON file
        self.static_obstacles: Dict[str, Obstacle] = {}
        
        # Flag for first crate update logging
        self.first_crate_update = False
        
        # Create publishers
        self.current_obs_pub = node.create_publisher(
            ObstacleArray, '/obstacles/current', 10)
        
        self.path_viz_pub = node.create_publisher(
            MarkerArray, '/path_visualization', 10)
        
        self.relative_obstacle_marker_pub = node.create_publisher(
            MarkerArray, '/relative_obstacle_marker', 10)
        
        # Create subscribers
        node.create_subscription(
            Empty, '/obstacles/clear_all', self.handle_clear_all, 10)
        
        node.create_subscription(
            PoseStamped, '/obstacle_relative', self.handle_obstacle_relative, 10)
        
        # Create timer for publishing current obstacles
        obstacle_hz = params['rates']['obstacle_list_hz']
        node.create_timer(1.0 / obstacle_hz, self.publish_current_obstacles)
        
        self.node.get_logger().info('🗺️  Map Manager initialized')
    
    def handle_clear_all(self, msg: Empty):
        """Clear all obstacles"""
        with self.obstacles_lock:
            count = len(self.obstacles)
            self.obstacles.clear()
            self.obstacle_sources.clear()
            self.node.get_logger().info(f"🗑️  Cleared all {count} obstacles")
        # Publish obstacle visualization after clearing
        self.publish_obstacles(self.get_active_obstacles())
    
    def handle_obstacle_relative(self, msg: PoseStamped):
        """Handle incoming obstacle from /obstacle_relative topic (only keeps one enemy)"""
        with self.obstacles_lock:
            # Remove all existing enemy obstacles first (only keep one enemy)
            self.obstacles = [obs for obs in self.obstacles if obs.color != 'enemy']
            self.obstacle_sources = {k: v for k, v in self.obstacle_sources.items() if v != 'enemy'}
            
            # Convert position from meters to millimeters
            center_x_mm = msg.pose.position.x * 1000.0
            center_y_mm = msg.pose.position.y * 1000.0
            
            # Create new enemy obstacle
            obs = Obstacle()
            obs.id = "enemy_robot"  # Fixed ID for the single enemy
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
        
        # Publish enemy marker to separate topic
        self._publish_enemy_marker(center_x_mm, center_y_mm, obs)
    
    def _publish_enemy_marker(self, center_x_mm: float, center_y_mm: float, obs: Obstacle):
        """Publish enemy robot visualization marker"""
        marker_array = MarkerArray()
        
        enemy_marker = Marker()
        enemy_marker.header.frame_id = "map"
        enemy_marker.header.stamp = self.node.get_clock().now().to_msg()
        enemy_marker.ns = "enemy"
        enemy_marker.id = 0
        enemy_marker.type = Marker.CUBE
        enemy_marker.action = Marker.ADD
        
        # Position (center of enemy robot)
        enemy_marker.pose.position.x = center_x_mm / 1000.0
        enemy_marker.pose.position.y = center_y_mm / 1000.0
        enemy_marker.pose.position.z = 0.15  # 150mm high (half of 300mm)
        enemy_marker.pose.orientation = self.yaw_to_quaternion(obs.pose.theta)
        
        # Size
        enemy_marker.scale.x = obs.width_mm / 1000.0
        enemy_marker.scale.y = obs.height_mm / 1000.0
        enemy_marker.scale.z = 0.30  # 300mm tall robot
        
        # Dark red/magenta for enemy
        enemy_marker.color.r = 0.8
        enemy_marker.color.g = 0.0
        enemy_marker.color.b = 0.4
        enemy_marker.color.a = 0.9
        
        # Make marker persistent
        enemy_marker.lifetime.sec = 2
        enemy_marker.lifetime.nanosec = 0
        
        marker_array.markers.append(enemy_marker)
        self.relative_obstacle_marker_pub.publish(marker_array)
    
    def publish_current_obstacles(self):
        """Periodically publish current obstacle list"""
        with self.obstacles_lock:
            msg = ObstacleArray()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.obstacles = list(self.obstacles)
            self.current_obs_pub.publish(msg)
        # Also publish obstacle visualization for RViz
        self.publish_obstacles(self.get_active_obstacles())
    
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
    
    def publish_obstacles(self, obstacles: List[dict]):
        """Publish obstacle visualization for RViz"""
        marker_array = MarkerArray()
        
        # First, send a DELETE ALL marker to clear old markers
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.node.get_clock().now().to_msg()
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
            crate_marker.header.stamp = self.node.get_clock().now().to_msg()
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
    
    def handle_update_map(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Service handler: Update map by requesting stable batches from ArUco mapper or web handler
        Calls /get_stable_batches service and processes the returned batches
        Uses 'sfg' filter to get all stable batches
        
        Priority: Try web service first (for simulation), then fall back to aruco_mapper (for real robot)
        """
        self.node.get_logger().info("🗺️  Update map service called")
        
        # Try web service first if available
        client_to_use = None
        service_name = ""
        
        
        if self.get_stable_batches_client.service_is_ready():
            client_to_use = self.get_stable_batches_client
            service_name = "/aruco_mapper/get_stable_batches"
            self.node.get_logger().info("📡 Using aruco_mapper service (real robot mode)")
        elif self.get_stable_batches_client_web and self.get_stable_batches_client_web.service_is_ready():
            client_to_use = self.get_stable_batches_client_web
            service_name = "/aruco_mapper/get_stable_batches_web"
            self.node.get_logger().info("📡 Using web handler service (simulation mode)")
        else:
            self.node.get_logger().error("❌ No GetStableBatches service available")
            response.success = False
            response.message = "No batch service available"
            return response
        
        # Create request for GetStableBatches with 'sfg' filter (all batches)
        stable_request = GetStableBatches.Request()
        stable_request.batch_types_filter = "sfg"  # Get all stable batches
        stable_request.specific_known_id = ""  # Get all batches matching the filter
        
        # Call service asynchronously and register callback with response object
        self.node.get_logger().info(f"📡 Calling {service_name} service...")
        future = client_to_use.call_async(stable_request)
        future.add_done_callback(lambda f: self._handle_get_stable_batches_response(f, response))
        
        # Return immediately - response will be populated by callback
        # Note: This is a limitation - service returns before processing completes
        # But it avoids the deadlock issue
        response.success = True
        response.message = f"Map update triggered from {service_name} (processing asynchronously)"
        return response
    
    def _handle_get_stable_batches_response(self, future, trigger_response=None):
        """
        Callback for GetStableBatches service response
        Processes the received batches and updates the map
        
        Args:
            future: Future object containing the service response
            trigger_response: Optional Trigger response (not used currently due to async nature)
        """
        try:
            stable_response = future.result()
            
            if stable_response is None:
                self.node.get_logger().error("❌ Service returned None")
                return

            # Process the received ArucoMapperState
            aruco_state = stable_response.stable_state
            num_batches = len(aruco_state.batches)
            
            self.node.get_logger().info(f"✅ Received {num_batches} batches from ArUco mapper")
            
            # Filter batches by state: only STABLE_VALID and OBSTRUCTED
            filtered_batches = []
            for batch in aruco_state.batches:
                state = batch.state.upper()
                if state in ['TRACKING', 'DEGRADED','OCCLUDED']:
                    filtered_batches.append(batch)

            self.node.get_logger().info(
                f"📊 Filtered {len(filtered_batches)}/{num_batches} batches "
                f"(TRACKING or EGRADED or occuluded only)"
            )
            
            # Deduplicate batches by known_id - keep the most complete one
            # The ArUco service may return the same batch multiple times with different data completeness
            deduplicated_batches = {}
            for batch in filtered_batches:
                known_id = batch.known_id
                if known_id not in deduplicated_batches:
                    deduplicated_batches[known_id] = batch
                else:
                    existing = deduplicated_batches[known_id]
                    # Keep the batch with more members, or if same, the one with longer pattern
                    if len(batch.members) > len(existing.members):
                        deduplicated_batches[known_id] = batch
                        self.node.get_logger().info(
                            f"🔄 Batch '{known_id}': replacing {len(existing.members)} members "
                            f"with {len(batch.members)} members"
                        )
                    elif len(batch.members) == len(existing.members) and len(batch.pattern) > len(existing.pattern):
                        deduplicated_batches[known_id] = batch
                        self.node.get_logger().info(
                            f"🔄 Batch '{known_id}': replacing pattern '{existing.pattern}' "
                            f"with '{batch.pattern}'"
                        )
            
            filtered_batches = list(deduplicated_batches.values())
            self.node.get_logger().info(f"✅ After deduplication: {len(filtered_batches)} unique batches")
            
            # Determine source - for 'sfg' we process both SX and FX
            # Process SX batches (s prefix)
            sx_batches = [b for b in filtered_batches if b.known_id.startswith('s')]
            fx_batches = [b for b in filtered_batches if b.known_id.startswith('f')]
            gx_batches = [b for b in filtered_batches if b.known_id.startswith('g')]
            
            # All other batches (web batches with custom names) should be treated as FX
            other_batches = [b for b in filtered_batches if not (b.known_id.startswith('s') or b.known_id.startswith('f') or b.known_id.startswith('g'))]
            if other_batches:
                self.node.get_logger().info(f"🌐 Found {len(other_batches)} web batches with custom names, treating as FX")
                fx_batches.extend(other_batches)
            
            obstacles_before = len(self.obstacles)
            
            # Process SX batches
            if sx_batches:
                sx_state = ArucoMapperState()
                sx_state.stamp = aruco_state.stamp
                sx_state.initialized = aruco_state.initialized
                sx_state.batches = sx_batches
                self._handle_aruco_mapper_state(sx_state, 'sx')
                self.node.get_logger().info(f"✅ Processed {len(sx_batches)} SX batches")
            
            # Process FX batches
            if fx_batches:
                fx_state = ArucoMapperState()
                fx_state.stamp = aruco_state.stamp
                fx_state.initialized = aruco_state.initialized
                fx_state.batches = fx_batches
                self._handle_aruco_mapper_state(fx_state, 'fx')
                self.node.get_logger().info(f"✅ Processed {len(fx_batches)} FX batches")
            
            # Process GX batches (goal area)
            if gx_batches:
                gx_state = ArucoMapperState()
                gx_state.stamp = aruco_state.stamp
                gx_state.initialized = aruco_state.initialized
                gx_state.batches = gx_batches
                self._handle_aruco_mapper_state(gx_state, 'gx')
                self.node.get_logger().info(f"✅ Processed {len(gx_batches)} GX batches")
            
            # Calculate obstacles created
            obstacles_after = len(self.obstacles)
            num_obstacles = obstacles_after - obstacles_before
            
            self.node.get_logger().info(
                f"✅ Map updated: processed {len(filtered_batches)} batches "
                f"(DEGRADED/TRACKING), created {num_obstacles} obstacles"
            )
            
        except Exception as e:
            self.node.get_logger().error(f"❌ Error processing service response: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())
    
    def _handle_aruco_mapper_state(self, msg: ArucoMapperState, source: str):
        """
        Internal method to handle ArUco mapper state updates
        source: 'fx' or 'sx'
        """
        self.first_crate_update=True
        with self.obstacles_lock:
            # Remove only obstacles from this source
            old_count = len(self.obstacles)
            self.obstacles = [obs for obs in self.obstacles if self.obstacle_sources.get(obs.id, '') != source]
            removed = old_count - len(self.obstacles)
            
            # Clear obstacle sources for this source
            self.obstacle_sources = {k: v for k, v in self.obstacle_sources.items() if v != source}

            # Process each batch
            for batch in msg.batches:
                # For FX batches, skip if no members
                # For SX batches, we use predefined positions, so members are not required
                if source == 'fx' and not batch.members:
                    continue
                
                # Store batch by known_id in the appropriate dictionary FIRST (always store, even if can't create obstacles)
                with self.batches_lock:
                    if source == 'fx':
                        self.batches_fx[batch.known_id] = batch
                    else:
                        self.batches_sx[batch.known_id] = batch
                
                if source == 'sx':
                    # SX: Use predefined batch position from lookup table
                    batch_known_id = batch.known_id
                    
                    # DEBUG: Log what we received from SX
                    self.node.get_logger().info(
                        f"🔍 SX Batch '{batch_known_id}' received: "
                        f"pattern='{batch.pattern}', direction='{batch.direction}', "
                        f"count={batch.count}, members={len(batch.members)}"
                    )
                    
                    if batch_known_id not in self.predefined_batch_positions:
                        self.node.get_logger().warn(f"⚠️  Batch '{batch_known_id}' not found in predefined positions, skipping obstacle creation")
                        continue  # Skip obstacle creation, but batch is already stored
                    
                    batch_pos = self.predefined_batch_positions[batch_known_id]
                    batch_center_x = batch_pos["x"]  # Already in meters
                    batch_center_y = batch_pos["y"]  # Already in meters
                    
                    # Update batch center with predefined position
                    # This is CRITICAL for distance calculations in handle_get_batch_color_pattern
                    batch.center.x = batch_center_x
                    batch.center.y = batch_center_y
                    batch.center.z = 0.0
                
                if source == 'sx':
                    # Continue with SX obstacle creation using predefined position
                    
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
                    
                    # Get color pattern from batch pattern string (e.g., "bbyy")
                    pattern_str = batch.pattern.lower() if batch.pattern else ""
                    
                    # If no pattern provided, skip obstacle creation for this batch
                    if not pattern_str:
                        self.node.get_logger().info(
                            f"ℹ️  Batch '{batch_known_id}' has no pattern field, skipping obstacle creation"
                        )
                        continue
                    
                    # Parse pattern string to extract colors
                    pattern_colors = []
                    for char in pattern_str:
                        if char == 'b':
                            pattern_colors.append('blue')
                        elif char == 'y':
                            pattern_colors.append('yellow')
                    
                    # If pattern exists but has no valid colors, skip
                    if not pattern_colors:
                        self.node.get_logger().warn(
                            f"⚠️  Batch '{batch_known_id}' has invalid pattern (pattern='{batch.pattern}'), "
                            f"skipping obstacle creation"
                        )
                        continue
                    
                    self.node.get_logger().info(f"✅ Creating 4 obstacles for batch '{batch_known_id}' with pattern {pattern_colors}")
                    
                    # Create 4 obstacles for this batch
                    for idx, (offset_x, offset_y) in enumerate(offsets):
                        obs = Obstacle()
                        obs.id = f"{source}_{batch_known_id}_member{idx}"  # Use known_id for unique IDs
                        obs.type = 'crate'
                        # Use color from pattern at this index (or last color if not enough in pattern)
                        obs.color = pattern_colors[idx] if idx < len(pattern_colors) else pattern_colors[-1]
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
                    # Parse pattern string (e.g., "ybyb", "yybb") to get colors
                    pattern_str = batch.pattern if batch.pattern else ""
                    pattern_colors = self.parse_pattern_colors(pattern_str)
                    
                    for idx, member in enumerate(batch.members):
                        # Create obstacle from batch member
                        obs = Obstacle()
                        obs.id = f"{source}_{batch.known_id}_member{idx}"  # Use known_id for unique IDs
                        obs.type = 'crate'
                        
                        # Use color from pattern if available, otherwise from member
                        if pattern_colors and idx < len(pattern_colors):
                            obs.color = pattern_colors[idx]
                        elif member.color in ["yellow", "blue"]:
                            obs.color = member.color
                        else:
                            # Skip if no valid color
                            continue

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

                        self.node.get_logger().info(f"  ✅ Created obstacle '{obs.id}' at ({center_x_mm:.0f}, {center_y_mm:.0f}) mm, color={obs.color}")
                        # Add to obstacles list and track source
                        self.obstacles.append(obs)
                        self.obstacle_sources[obs.id] = source
            
            # Count obstacles by source
            fx_count = sum(1 for v in self.obstacle_sources.values() if v == 'fx')
            sx_count = sum(1 for v in self.obstacle_sources.values() if v == 'sx')
            enemy_count = sum(1 for v in self.obstacle_sources.values() if v == 'enemy')
            self.node.get_logger().info(
                f"📦 Updated {source.upper()} obstacles from {len(msg.batches)} batches | "
                f"Total obstacles: FX={fx_count}, SX={sx_count}, Enemy={enemy_count}")

    def get_batch_by_known_id(self, known_id: str) -> Optional[Batch]:
        """Get batch by known_id from either FX or SX storage"""
        with self.batches_lock:
            # Check FX batches
            if known_id in self.batches_fx:
                return self.batches_fx[known_id]
            # Check SX batches
            if known_id in self.batches_sx:
                return self.batches_sx[known_id]
            return None
    
    def get_all_batches(self) -> Dict[str, Batch]:
        """Get all batches from both FX and SX"""
        with self.batches_lock:
            return {**self.batches_fx, **self.batches_sx}
    
    def get_obstacle_counts(self) -> Dict[str, int]:
        """Get counts of obstacles by source"""
        with self.obstacles_lock:
            fx_count = sum(1 for v in self.obstacle_sources.values() if v == 'fx')
            sx_count = sum(1 for v in self.obstacle_sources.values() if v == 'sx')
            enemy_count = sum(1 for v in self.obstacle_sources.values() if v == 'enemy')
            return {'fx': fx_count, 'sx': sx_count, 'enemy': enemy_count}
    
    @staticmethod
    def parse_pattern_colors(pattern_str: str) -> List[str]:
        """Parse pattern string (e.g., 'ybyb', 'yybb') into color list"""
        pattern_colors = []
        for char in pattern_str.lower():
            if char == 'b':
                pattern_colors.append('blue')
            elif char == 'y':
                pattern_colors.append('yellow')
        return pattern_colors
    
    @staticmethod
    def quaternion_to_yaw(quat: Quaternion) -> float:
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat
