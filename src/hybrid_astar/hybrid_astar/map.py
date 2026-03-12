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
        
        # Flag to track if this is the first update_map call
        self.first_update_map = True
        
        # Flag to track if we've locked the baseline patterns
        self.baselines_locked = False
        
        # Set to track known_ids that need to be retried (not in TRACKING state)
        self.known_ids_to_retry: set = set()
        
        # Set to track manually deleted batch known_ids (should not be re-added)
        self.deleted_batch_ids: set = set()
        
        # Set to track currently active batch known_ids (from last update)
        self.active_batch_ids: set = set()
        
        # Set to track batches that already have obstacles created (prevents duplicates)
        self.batches_with_obstacles: set = set()

        # Retry management for auto update_map
        self.retry_timer = None
        self.retry_interval_sec = params.get('rates', {}).get('update_map_retry_sec', 1.0)
        self.retry_pending = 0
        self.retry_client = None
        self.retry_service_name = ""
        
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
        
        # Clear batch obstacle tracking
        with self.batches_lock:
            self.batches_with_obstacles.clear()
        
        # Publish obstacle visualization after clearing
        self.publish_obstacles(self.get_active_obstacles())
    
    def handle_clear_map(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Service handler: Clear all obstacles and reset map state for fresh update"""
        self.node.get_logger().info("🗺️  Clear map service called")
        
        # Stop any running retry timer
        self._stop_retry_timer()
        
        # Clear all obstacles
        with self.obstacles_lock:
            obstacle_count = len(self.obstacles)
            self.obstacles.clear()
            self.obstacle_sources.clear()
        
        # Clear all batches
        with self.batches_lock:
            fx_count = len(self.batches_fx)
            sx_count = len(self.batches_sx)
            self.batches_fx.clear()
            self.batches_sx.clear()
            self.batches_with_obstacles.clear()  # Clear tracking
        
        # Reset state flags
        self.first_update_map = True
        self.baselines_locked = False
        self.known_ids_to_retry.clear()
        self.deleted_batch_ids.clear()
        self.active_batch_ids.clear()
        
        # Publish empty obstacle visualization
        self.publish_obstacles([])
        
        self.node.get_logger().info(
            f"✅ Map cleared: {obstacle_count} obstacles, {fx_count} FX batches, {sx_count} SX batches removed. "
            f"State reset for fresh update_map call."
        )
        
        response.success = True
        response.message = f"Map cleared: {obstacle_count} obstacles removed, state reset"
        return response
    
    def delete_batch(self, known_id: str) -> bool:
        """
        Manually delete a batch and mark it to not be re-added
        
        Args:
            known_id: The known_id of the batch to delete (e.g., 's1', 'f_batch_1')
        
        Returns:
            True if batch was found and deleted, False otherwise
        """
        self.node.get_logger().info(f"🗑️  Manually deleting batch '{known_id}'")
        
        deleted = False
        
        # Remove obstacles for this batch
        with self.obstacles_lock:
            old_count = len(self.obstacles)
            self.obstacles = [obs for obs in self.obstacles 
                            if not (obs.id.startswith(f"fx_{known_id}_") or obs.id.startswith(f"sx_{known_id}_"))]
            removed = old_count - len(self.obstacles)
            
            if removed > 0:
                deleted = True
                # Remove from obstacle sources
                self.obstacle_sources = {k: v for k, v in self.obstacle_sources.items() 
                                        if not (k.startswith(f"fx_{known_id}_") or k.startswith(f"sx_{known_id}_"))}
                self.node.get_logger().info(f"  ✅ Removed {removed} obstacles for batch '{known_id}'")
        
        # Remove from batch dictionaries
        with self.batches_lock:
            if known_id in self.batches_fx:
                del self.batches_fx[known_id]
                deleted = True
                self.node.get_logger().info(f"  ✅ Removed FX batch '{known_id}'")
            if known_id in self.batches_sx:
                del self.batches_sx[known_id]
                deleted = True
                self.node.get_logger().info(f"  ✅ Removed SX batch '{known_id}'")
        
        # Add to deleted set to prevent re-addition
        self.deleted_batch_ids.add(known_id)
        
        # Remove from active batch IDs
        self.active_batch_ids.discard(known_id)
        
        # Remove from retry list if present
        self.known_ids_to_retry.discard(known_id)
        
        # Publish updated obstacles
        self.publish_obstacles(self.get_active_obstacles())
        
        if deleted:
            self.node.get_logger().info(
                f"✅ Batch '{known_id}' deleted and marked to not be re-added. "
                f"Deleted batches: {self.deleted_batch_ids}"
            )
        else:
            self.node.get_logger().warn(f"⚠️  Batch '{known_id}' not found, but marked as deleted")
        
        return deleted
    
    def undelete_batch(self, known_id: str) -> bool:
        """
        Remove a batch from the deleted list, allowing it to be added again on next update
        
        Args:
            known_id: The known_id of the batch to undelete
        
        Returns:
            True if batch was in deleted list, False otherwise
        """
        if known_id in self.deleted_batch_ids:
            self.deleted_batch_ids.remove(known_id)
            self.node.get_logger().info(
                f"✅ Batch '{known_id}' removed from deleted list. "
                f"Will be added on next update if beacon provides it. "
                f"Deleted batches: {self.deleted_batch_ids}"
            )
            return True
        else:
            self.node.get_logger().warn(f"⚠️  Batch '{known_id}' was not in deleted list")
            return False
    
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
            
            # Enemy robot represented as a circle with diameter 0.6m (600mm)
            enemy_diameter_mm = 600.0
            enemy_radius_mm = enemy_diameter_mm / 2.0  # 300
            # For collision checking, represent circle as a square with side = radius (300mm)
            obs.width_mm = enemy_radius_mm
            obs.height_mm = enemy_radius_mm
            
            # Convert to top-left corner (what the planner expects)
            # For a square centered at (center_x, center_y), top-left = center - (width/2)
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
        """Publish enemy robot visualization marker as a cylinder (circle)"""
        marker_array = MarkerArray()
        
        enemy_marker = Marker()
        enemy_marker.header.frame_id = "map"
        enemy_marker.header.stamp = self.node.get_clock().now().to_msg()
        enemy_marker.ns = "enemy"
        enemy_marker.id = 0
        enemy_marker.type = Marker.CYLINDER  # Cylinder to represent circular robot
        enemy_marker.action = Marker.ADD
        
        # Position (center of enemy robot)
        enemy_marker.pose.position.x = center_x_mm / 1000.0
        enemy_marker.pose.position.y = center_y_mm / 1000.0
        enemy_marker.pose.position.z = 0.15  # 150mm high (half of 300mm)
        enemy_marker.pose.orientation = self.yaw_to_quaternion(0.0)  # Cylinders don't need rotation
        
        # Size: 0.30m square (300mm radius)
        enemy_marker.scale.x = 0.30  # 300mm side in X
        enemy_marker.scale.y = 0.30  # 300mm side in Y
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
        
        First call: Uses 'sfg' filter to get all batches, processes only TRACKING ones
        Subsequent calls: Retries specific known_ids that were not TRACKING
        
        Priority: Try web service first (for simulation), then fall back to aruco_mapper (for real robot)
        """
        # Try web service first if available
        client_to_use = None
        service_name = ""
        
        if self.get_stable_batches_client.service_is_ready():
            client_to_use = self.get_stable_batches_client
            service_name = "/aruco_mapper/get_stable_batches"
        elif self.get_stable_batches_client_web and self.get_stable_batches_client_web.service_is_ready():
            client_to_use = self.get_stable_batches_client_web
            service_name = "/aruco_mapper/get_stable_batches_web"
        else:
            self.node.get_logger().error("❌ No GetStableBatches service available")
            response.success = False
            response.message = "No batch service available"
            return response

        # Remember service for auto-retry timer
        self.retry_client = client_to_use
        self.retry_service_name = service_name
        
        # Always get all batches with 'sfg' filter (for both first and subsequent calls)
        # The difference is that first call clears everything, subsequent calls do incremental updates
        if self.first_update_map:
            self.node.get_logger().info("🔄 First update_map call - requesting all batches (unlocked)")
        
        stable_request = GetStableBatches.Request()
        stable_request.batch_types_filter = "sfg"  # Get all stable batches
        stable_request.specific_known_id = ""  # Get all batches matching the filter
        stable_request.only_locked_baselines = False  # Always False for update_map - lock call happens after all TRACKING
        
        # Call service asynchronously
        future = client_to_use.call_async(stable_request)
        future.add_done_callback(lambda f: self._handle_get_stable_batches_response(
            f, response, client_to_use, service_name, None, False))
        
        response.success = True
        message_type = "First" if self.first_update_map else "Incremental"
        response.message = f"{message_type} map update triggered from {service_name} (processing asynchronously)"
        return response

    def handle_reset_map(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Service handler: Reset map by requesting stable batches filtered by 'rrr'.
        Works identically to handle_update_map but uses 'rrr' as the batch_types_filter.
        """
        client_to_use = None
        service_name = ""

        if self.get_stable_batches_client.service_is_ready():
            client_to_use = self.get_stable_batches_client
            service_name = "/aruco_mapper/get_stable_batches"
        elif self.get_stable_batches_client_web and self.get_stable_batches_client_web.service_is_ready():
            client_to_use = self.get_stable_batches_client_web
            service_name = "/aruco_mapper/get_stable_batches_web"
        else:
            self.node.get_logger().error("❌ No GetStableBatches service available")
            response.success = False
            response.message = "No batch service available"
            return response

        self.retry_client = client_to_use
        self.retry_service_name = service_name

        self.node.get_logger().info("🔄 reset_map called - requesting batches with 'rrr' filter")

        stable_request = GetStableBatches.Request()
        stable_request.batch_types_filter = "rrr"
        stable_request.specific_known_id = ""
        stable_request.only_locked_baselines = False

        future = client_to_use.call_async(stable_request)
        future.add_done_callback(lambda f: self._handle_get_stable_batches_response(
            f, response, client_to_use, service_name, None, False))

        response.success = True
        response.message = f"Reset map triggered from {service_name} with 'rrr' filter (processing asynchronously)"
        return response

    def _handle_get_stable_batches_response(self, future, trigger_response=None, client=None, service_name="", specific_known_id=None, from_retry_timer=False):
        """
        Callback for GetStableBatches service response
        Processes the received batches and updates the map
        
        Args:
            future: Future object containing the service response
            trigger_response: Optional Trigger response (not used currently due to async nature)
            client: Service client for retrying specific known_ids
            service_name: Name of the service being used
            specific_known_id: If set, this is a retry for a specific known_id
        """
        try:
            stable_response = future.result()
            
            if stable_response is None:
                self.node.get_logger().error("❌ Service returned None")
                return

            # Process the received ArucoMapperState
            aruco_state = stable_response.stable_state
            num_batches = len(aruco_state.batches)
            
            # Initialize variables that may be used in final summary
            disappeared_ids = set()
            
            # Remember if this is the first update (before we set it to False)
            is_first_update = self.first_update_map and specific_known_id is None
            is_full_update = specific_known_id is None  # Full update (not a specific retry)
            
            # Filter batches based on update type
            if is_first_update:
                # FIRST UPDATE: accept non-EMERGING batches, queue EMERGING SX/GX for retry
                filtered_batches = []
                emerging_ids = set()
                non_emerging_count = 0
                skipped_count = 0
                
                for batch in aruco_state.batches:
                    state = batch.state.upper()
                    if state != 'EMERGING':
                        # Accept all non-EMERGING batches (TRACKING, DEGRADED, etc.)
                        filtered_batches.append(batch)
                        non_emerging_count += 1
                    else:
                        # Only retry EMERGING SX/GX batches
                        if batch.known_id.startswith('s') or batch.known_id.startswith('g'):
                            emerging_ids.add(batch.known_id)
                            batch_type = 'SX' if batch.known_id.startswith('s') else 'GX'
                            self.node.get_logger().info(f"  ⏳ [{batch_type}] '{batch.known_id}': {state} → will retry")
                        else:
                            skipped_count += 1
                
                # Store EMERGING IDs for retry (only SX and GX)
                self.known_ids_to_retry = emerging_ids
                
                self.node.get_logger().info(
                    f"📊 First update summary: {non_emerging_count} non-EMERGING, {len(emerging_ids)} SX/GX EMERGING queued for retry, "
                    f"{skipped_count} FX skipped"
                )
                
                # Mark first update as complete
                self.first_update_map = False

                # Start auto-retry timer if needed
                if self.known_ids_to_retry:
                    self._start_retry_timer()
                    
            elif is_full_update:
                # SUBSEQUENT FULL UPDATE: process non-EMERGING batches, retry EMERGING SX/GX
                filtered_batches = []
                emerging_ids = set()
                non_emerging_count = 0
                skipped_count = 0
                
                for batch in aruco_state.batches:
                    state = batch.state.upper()
                    is_fx = batch.known_id.startswith('f') or not (batch.known_id.startswith('s') or batch.known_id.startswith('g'))
                    is_sx_gx = batch.known_id.startswith('s') or batch.known_id.startswith('g')
                    batch_type = 'FX' if is_fx else ('SX' if batch.known_id.startswith('s') else 'GX')
                    
                    if state != 'EMERGING':
                        # Accept all non-EMERGING batches (TRACKING, DEGRADED, etc.)
                        filtered_batches.append(batch)
                        non_emerging_count += 1
                        # Remove from retry list if it was there (no longer EMERGING)
                        if batch.known_id in self.known_ids_to_retry:
                            self.known_ids_to_retry.discard(batch.known_id)
                            self.node.get_logger().info(f"  🔄 [{batch_type}] '{batch.known_id}': {state} (no longer EMERGING)")
                    else:
                        # Handle EMERGING batches
                        if state == 'LOST':
                            # LOST: Remove from retry queue and skip (will be removed as disappeared)
                            if batch.known_id in self.known_ids_to_retry:
                                self.known_ids_to_retry.discard(batch.known_id)
                                self.node.get_logger().info(f"  🗑️  [{batch_type}] '{batch.known_id}': LOST → removed from retry")
                            skipped_count += 1
                        elif is_sx_gx:
                            # EMERGING SX/GX: Add to retry queue (log only on first add)
                            if batch.known_id not in emerging_ids and batch.known_id not in self.known_ids_to_retry:
                                self.node.get_logger().info(f"  ⏭️  [{batch_type}] '{batch.known_id}': EMERGING → added to retry")
                            emerging_ids.add(batch.known_id)
                            skipped_count += 1
                        else:
                            # EMERGING FX - skip
                            skipped_count += 1
                
                # Update retry list (add new EMERGING batches, keep existing ones)
                self.known_ids_to_retry.update(emerging_ids)
                
                # Only log if there are batches in retry or state changes
                if self.known_ids_to_retry:
                    self.node.get_logger().info(
                        f"📊 Update: {non_emerging_count} non-EMERGING, {len(self.known_ids_to_retry)} EMERGING in retry queue"
                    )
                
                # Start auto-retry timer if needed
                if self.known_ids_to_retry:
                    self._start_retry_timer()
                    
            else:
                # SPECIFIC RETRY: check if specific batch is no longer EMERGING
                filtered_batches = []
                
                for batch in aruco_state.batches:
                    state = batch.state.upper()
                    is_fx = batch.known_id.startswith('f') or not (batch.known_id.startswith('s') or batch.known_id.startswith('g'))
                    batch_type = 'FX' if is_fx else ('SX' if batch.known_id.startswith('s') else 'GX')
                    
                    if state != 'EMERGING':
                        # Accept non-EMERGING batches
                        filtered_batches.append(batch)
                        # Remove from retry list if no longer EMERGING
                        if batch.known_id in self.known_ids_to_retry:
                            self.known_ids_to_retry.discard(batch.known_id)
                            self.node.get_logger().info(f"  🎉 [{batch_type}] '{batch.known_id}': {state} ({len(batch.members)} members) → no longer EMERGING, removing from retry")
                    elif state == 'LOST':
                        # LOST: Remove from retry queue
                        self.known_ids_to_retry.discard(batch.known_id)
                        self.node.get_logger().info(f"  🗑️  [{batch_type}] '{batch.known_id}': {state} ({len(batch.members)} members) → LOST, removing from retry")
                    else:
                        # Still EMERGING
                        self.node.get_logger().info(f"  ⏳ [{batch_type}] '{batch.known_id}': {state} ({len(batch.members)} members) → still EMERGING, keeping in retry")
                
                if specific_known_id and len(filtered_batches) == 0:
                    batch_type = 'FX' if specific_known_id.startswith('f') else ('SX' if specific_known_id.startswith('s') else 'GX')
                    self.node.get_logger().info(f"  ⏳ [{batch_type}] '{specific_known_id}': Not received, will retry again")

            # Deduplicate batches by known_id - keep the most complete one
            # The ArUco service may return the same batch multiple times with different data completeness
            deduplicated_batches = {}
            duplicates_found = False
            for batch in filtered_batches:
                known_id = batch.known_id
                if known_id not in deduplicated_batches:
                    deduplicated_batches[known_id] = batch
                else:
                    duplicates_found = True
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
            if duplicates_found:
                self.node.get_logger().info(f"✅ After deduplication: {len(filtered_batches)} unique batches")
            
            # Get known_ids from current response
            received_batch_ids = {batch.known_id for batch in filtered_batches}
            
            # CLEAR ALL FX/SX BATCHES AND OBSTACLES ONLY ON FIRST UPDATE
            # On subsequent updates, we do incremental updates
            if specific_known_id is None:  # Only clear on first update (not on retries)
                if is_first_update:
                    # FIRST UPDATE: Clear everything
                    with self.obstacles_lock:
                        old_count = len(self.obstacles)
                        self.obstacles = [obs for obs in self.obstacles if self.obstacle_sources.get(obs.id, '') not in ['fx', 'sx']]
                        removed = old_count - len(self.obstacles)
                        
                        # Clear obstacle sources for both FX and SX
                        self.obstacle_sources = {k: v for k, v in self.obstacle_sources.items() if v not in ['fx', 'sx']}
                        
                        # Clear both batch dictionaries
                        with self.batches_lock:
                            self.batches_fx.clear()
                            self.batches_sx.clear()
                            self.batches_with_obstacles.clear()  # Clear tracking
                            self.node.get_logger().info(f"🗑️  Cleared all FX and SX batches and {removed} obstacles before first update")
                else:
                    # SUBSEQUENT UPDATES: Incremental update
                    # 1. Find batches that disappeared (in active but not in received)
                    disappeared_ids = self.active_batch_ids - received_batch_ids
                    
                    # 2. Remove obstacles ONLY for batches that disappeared OR are FX (FX always refreshed)
                    # Keep existing obstacles for SX batches that are still present
                    fx_batches_ids = {batch.known_id for batch in filtered_batches if batch.known_id.startswith('f')}
                    batches_to_remove = disappeared_ids | fx_batches_ids  # Disappeared + all FX
                    
                    if batches_to_remove:
                        with self.obstacles_lock:
                            old_count = len(self.obstacles)
                            self.obstacles = [obs for obs in self.obstacles 
                                            if not any(obs.id.startswith(f"fx_{bid}_") or obs.id.startswith(f"sx_{bid}_") 
                                                      for bid in batches_to_remove)]
                            removed = old_count - len(self.obstacles)
                            
                            # Remove from obstacle sources
                            self.obstacle_sources = {k: v for k, v in self.obstacle_sources.items() 
                                                    if not any(k.startswith(f"fx_{bid}_") or k.startswith(f"sx_{bid}_") 
                                                              for bid in batches_to_remove)}
                            
                            # Remove from batch dictionaries
                            with self.batches_lock:
                                for bid in disappeared_ids:
                                    # Remove disappeared batches from storage
                                    self.batches_fx.pop(bid, None)
                                    self.batches_sx.pop(bid, None)
                                    self.batches_with_obstacles.discard(bid)  # Remove from tracking
                                for bid in fx_batches_ids:
                                    # Remove FX batches for refresh (they'll be re-added)
                                    self.batches_fx.pop(bid, None)
                                    self.batches_with_obstacles.discard(bid)  # Remove from tracking
                            
                            # Log removal
                            if disappeared_ids and fx_batches_ids:
                                self.node.get_logger().info(
                                    f"🗑️  Removed {removed} obstacles: {len(disappeared_ids)} disappeared, {len(fx_batches_ids)} FX to refresh"
                                )
                            elif disappeared_ids:
                                self.node.get_logger().info(
                                    f"🗑️  Removed {removed} obstacles from {len(disappeared_ids)} disappeared batches: {disappeared_ids}"
                                )
                            elif fx_batches_ids:
                                self.node.get_logger().info(
                                    f"🔄 Refreshing {len(fx_batches_ids)} FX batches (removed {removed} old obstacles)"
                                )
                    else:
                        # No batches to remove, initialize empty set for consistency
                        fx_batches_ids = set()
                    
                    # 3. Filter out manually deleted batches (don't re-add them)
                    if self.deleted_batch_ids:
                        pre_filter_count = len(filtered_batches)
                        filtered_batches = [b for b in filtered_batches if b.known_id not in self.deleted_batch_ids]
                        filtered_count = pre_filter_count - len(filtered_batches)
                        if filtered_count > 0:
                            self.node.get_logger().info(
                                f"🚫 Filtered out {filtered_count} manually deleted batches: "
                                f"{self.deleted_batch_ids & received_batch_ids}"
                            )
                    
                    # 4. Filter out already-tracked SX/GX batches (keep existing obstacles, only add new ones)
                    # FX batches are always processed (refreshed every update)
                    new_batches = [b for b in filtered_batches 
                                   if b.known_id not in self.active_batch_ids or b.known_id.startswith('f')]
                    if len(new_batches) < len(filtered_batches):
                        kept_count = len(filtered_batches) - len(new_batches)
                        self.node.get_logger().info(
                            f"♻️  Keeping {kept_count} existing SX/GX batches, adding {len(new_batches)} new/FX ones"
                        )
                    filtered_batches = new_batches
            else:
                # On retry: only remove obstacles if we have better data (TRACKING or new members)
                # For DEGRADED batches with no members, keep existing obstacles to avoid flickering
                should_update = False
                if filtered_batches:
                    for batch in filtered_batches:
                        if batch.known_id == specific_known_id:
                            # Remove obstacles only if TRACKING (upgrade) or if we have members (new data)
                            if batch.state.upper() == 'TRACKING' or (batch.members and len(batch.members) > 0):
                                should_update = True
                                with self.obstacles_lock:
                                    old_count = len(self.obstacles)
                                    self.obstacles = [obs for obs in self.obstacles 
                                                     if not (obs.id.startswith(f"fx_{specific_known_id}_") or obs.id.startswith(f"sx_{specific_known_id}_"))]
                                    removed = old_count - len(self.obstacles)
                                
                                # Clear from tracking so obstacles can be recreated
                                with self.batches_lock:
                                    self.batches_with_obstacles.discard(specific_known_id)
                            break
                
                # If no update needed, skip processing for this batch (keep existing obstacles)
                if not should_update and specific_known_id:
                    filtered_batches = []  # Clear to prevent reprocessing
            
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
            
            # Process FX batches
            if fx_batches:
                fx_state = ArucoMapperState()
                fx_state.stamp = aruco_state.stamp
                fx_state.initialized = aruco_state.initialized
                fx_state.batches = fx_batches
                self._handle_aruco_mapper_state(fx_state, 'fx')
            
            # Process GX batches (goal area)
            if gx_batches:
                gx_state = ArucoMapperState()
                gx_state.stamp = aruco_state.stamp
                gx_state.initialized = aruco_state.initialized
                gx_state.batches = gx_batches
                self._handle_aruco_mapper_state(gx_state, 'gx')
            
            # Calculate obstacles created
            obstacles_after = len(self.obstacles)
            num_obstacles = obstacles_after - obstacles_before
            
            # Update active batch IDs (only on full updates, not retries)
            if specific_known_id is None:
                # Update active batch IDs to current state
                # Use received_batch_ids (all batches in response) not filtered_batches (only new ones)
                self.active_batch_ids = received_batch_ids
                
                # Check if all batches are TRACKING and we haven't locked baselines yet
                if not self.baselines_locked and not self.known_ids_to_retry:
                    self._lock_baseline_patterns()
            
            # Count obstacles by type for summary
            fx_count = sum(1 for v in self.obstacle_sources.values() if v == 'fx')
            sx_count = sum(1 for v in self.obstacle_sources.values() if v == 'sx')
            enemy_count = sum(1 for v in self.obstacle_sources.values() if v == 'enemy')
            untracked_count = len(self.obstacles) - len(self.obstacle_sources)
            
            # Check for duplicate obstacle IDs (debugging)
            obs_ids = [obs.id for obs in self.obstacles]
            unique_ids = set(obs_ids)
            if len(obs_ids) != len(unique_ids):
                duplicates = [id for id in obs_ids if obs_ids.count(id) > 1]
                self.node.get_logger().warning(
                    f"⚠️  Found {len(obs_ids) - len(unique_ids)} duplicate obstacle IDs: {set(duplicates)}"
                )
            
            # Only log summary on first update or when changes occur
            if is_first_update or num_obstacles > 0 or disappeared_ids:
                if untracked_count > 0:
                    self.node.get_logger().warning(
                        f"📦 Map updated: FX={fx_count}, SX={sx_count}, Enemy={enemy_count}, Untracked={untracked_count}, Total={len(self.obstacles)}"
                    )
                else:
                    self.node.get_logger().info(
                        f"📦 Map updated: FX={fx_count}, SX={sx_count}, Enemy={enemy_count}, Total={len(self.obstacles)}"
                    )
            
        except Exception as e:
            self.node.get_logger().error(f"❌ Error processing service response: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())
        finally:
            if from_retry_timer:
                self.retry_pending = max(0, self.retry_pending - 1)
                if self.retry_pending == 0 and not self.known_ids_to_retry:
                    self._stop_retry_timer()

    def _start_retry_timer(self):
        """Start periodic retries until all EMERGING batches become non-EMERGING (stable)."""
        if self.retry_timer is not None:
            return

        interval = max(self.retry_interval_sec, 0.1)
        self.retry_timer = self.node.create_timer(interval, self._retry_timer_callback)
        self.node.get_logger().info(
            f"⏳ Auto-retry enabled: checking EMERGING batches every {interval:.1f}s until stable"
        )

    def _stop_retry_timer(self):
        """Stop periodic retries when all batches are TRACKING or removed."""
        if self.retry_timer is None:
            return

        self.retry_timer.cancel()
        self.retry_timer = None
        self.retry_pending = 0
        self.node.get_logger().info("✅ Auto-retry stopped: all batches are no longer EMERGING")
        
        # After all batches are no longer EMERGING, lock the baseline patterns
        if not self.baselines_locked:
            self._lock_baseline_patterns()
    
    def _lock_baseline_patterns(self):
        """Call GetStableBatches with only_locked_baselines=True to lock the baseline patterns."""
        if self.baselines_locked:
            return
        
        if not self.retry_client or not self.retry_client.service_is_ready():
            self.node.get_logger().warn("⚠️ Cannot lock baselines: batch service not ready")
            return
        
        self.node.get_logger().info("🔒 All batches are no longer EMERGING - locking baseline patterns...")
        
        stable_request = GetStableBatches.Request()
        stable_request.batch_types_filter = "sfg"
        stable_request.specific_known_id = ""
        stable_request.only_locked_baselines = True  # Request locked/immutable baselines
        
        future = self.retry_client.call_async(stable_request)
        future.add_done_callback(self._handle_lock_baseline_response)
    
    def _handle_lock_baseline_response(self, future):
        """Handle response from locked baseline patterns request."""
        try:
            response = future.result()
            if response and response.stable_state:
                num_batches = len(response.stable_state.batches)
                self.baselines_locked = True
                self.node.get_logger().info(f"✅ Baseline patterns locked! Received {num_batches} locked batches")
                
                # Track the locked batch IDs
                locked_batch_ids = set()
                
                # Update batches with locked patterns
                with self.batches_lock:
                    for batch in response.stable_state.batches:
                        locked_batch_ids.add(batch.known_id)
                        if batch.known_id.startswith('s'):
                            self.batches_sx[batch.known_id] = batch
                            self.node.get_logger().info(f"   🔒 Locked SX pattern: {batch.known_id} = '{batch.pattern}'")
                        elif batch.known_id.startswith('f'):
                            self.batches_fx[batch.known_id] = batch
                            self.node.get_logger().info(f"   🔒 Locked FX pattern: {batch.known_id} = '{batch.pattern}'")
                
                # CRITICAL: Add locked batches to active_batch_ids to prevent them from being removed
                self.active_batch_ids.update(locked_batch_ids)
                self.node.get_logger().info(f"   📌 Added {len(locked_batch_ids)} locked batches to active tracking")
            else:
                self.node.get_logger().error("❌ Failed to lock baseline patterns: no response")
        except Exception as e:
            self.node.get_logger().error(f"❌ Error locking baseline patterns: {e}")

    def _retry_timer_callback(self):
        """Timer callback to retry EMERGING known_ids."""
        if not self.known_ids_to_retry:
            self._stop_retry_timer()
            return

        if self.retry_pending > 0:
            return

        if not self.retry_client or not self.retry_client.service_is_ready():
            self.node.get_logger().warn("⚠️ Auto-retry: batch service not ready, will retry later")
            return

        self.node.get_logger().info(
            f"🔁 Auto-retrying {len(self.known_ids_to_retry)} EMERGING batches"
        )

        for known_id in list(self.known_ids_to_retry):
            stable_request = GetStableBatches.Request()
            stable_request.batch_types_filter = "sfg"
            stable_request.specific_known_id = known_id
            stable_request.only_locked_baselines = False  # Retries should not use locked baselines only

            future = self.retry_client.call_async(stable_request)
            self.retry_pending += 1
            future.add_done_callback(
                lambda f, kid=known_id: self._handle_get_stable_batches_response(
                    f, None, self.retry_client, self.retry_service_name, kid, True
                )
            )
    
    def _handle_aruco_mapper_state(self, msg: ArucoMapperState, source: str):
        """
        Internal method to handle ArUco mapper state updates
        source: 'fx', 'sx', or 'gx'
        Note: FX/SX batches and obstacles are cleared before this is called
        """
        self.first_crate_update=True
        with self.obstacles_lock:
            # Process each batch (no clearing needed - done at top level)
            for batch in msg.batches:
                # Skip GX batches (goal area markers - don't create obstacles)
                if source == 'gx':
                    continue
                
                # For FX batches, log member count and skip if no members
                # For SX batches, we use predefined positions, so members are not required
                if source == 'fx':
                    member_count = len(batch.members) if batch.members else 0
                    self.node.get_logger().info(
                        f"📦 FX batch '{batch.known_id}' received with {member_count} members"
                    )
                    if not batch.members:
                        continue
                
                # Store batch by known_id in the appropriate dictionary FIRST (always store, even if can't create obstacles)
                with self.batches_lock:
                    if source == 'fx':
                        self.batches_fx[batch.known_id] = batch
                    elif source == 'sx':
                        self.batches_sx[batch.known_id] = batch
                    # GX batches are not stored (goal area markers only)
                    
                    # Check if this batch already has obstacles created
                    if batch.known_id in self.batches_with_obstacles:
                        # Skip obstacle creation - already processed
                        self.node.get_logger().info(
                            f"⏭️  Skipping '{batch.known_id}' - obstacles already created"
                        )
                        skip_creation = True
                    else:
                        skip_creation = False
                
                if skip_creation:
                    continue
                
                if source == 'sx':
                    # SX: Use predefined batch position from lookup table
                    batch_known_id = batch.known_id
                    
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
                    
                    # Check if batch has members with real positions
                    if batch.members and len(batch.members) > 0 and len(batch.members) < 4:
                        # Use REAL member positions (for batches with < 4 members)
                        self.node.get_logger().info(
                            f"✅ Creating {len(batch.members)} obstacles for batch '{batch_known_id}' "
                            f"using REAL positions from members"
                        )
                        
                        for idx, member in enumerate(batch.members):
                            obs = Obstacle()
                            obs.id = f"{source}_{batch_known_id}_member{idx}"
                            obs.type = 'crate'
                            
                            # Use color from pattern if available
                            if idx < len(pattern_colors):
                                obs.color = pattern_colors[idx]
                            else:
                                obs.color = pattern_colors[-1] if pattern_colors else 'yellow'
                            
                            # Determine obstacle dimensions based on batch direction
                            if batch.direction == "horizontal":
                                obs.width_mm = 50.0
                                obs.height_mm = 150.0
                            elif batch.direction == "vertical":
                                obs.width_mm = 150.0
                                obs.height_mm = 50.0
                            else:
                                # Skip if no direction
                                self.node.get_logger().warn(
                                    f"⚠️  Batch '{batch_known_id}' has no direction, skipping"
                                )
                                continue
                            
                            # Use ACTUAL member position from service
                            center_x_mm = member.map_pos.x * 1000.0
                            center_y_mm = member.map_pos.y * 1000.0
                            
                            # Convert to TOP-LEFT corner
                            obs.pose.x = center_x_mm - (obs.width_mm / 2.0)
                            obs.pose.y = center_y_mm - (obs.height_mm / 2.0)
                            obs.pose.theta = 0.0
                            
                            obs.detected_at = msg.stamp
                            
                            self.node.get_logger().info(
                                f"  ✅ Created obstacle '{obs.id}' at REAL position "
                                f"({center_x_mm:.0f}, {center_y_mm:.0f}) mm, color={obs.color}"
                            )
                            
                            # Add to obstacles list and track source
                            self.obstacles.append(obs)
                            self.obstacle_sources[obs.id] = source
                    
                    else:
                        # Use PREDEFINED positions with offsets (for full batches with 4 members or no member data)
                        # Determine obstacle dimensions based on batch direction
                        if batch.direction == "horizontal":
                            obs_width_mm = 50.0
                            obs_height_mm = 150.0
                            offsets = [(-75.0, 0.0), (-25.0, 0.0), (25.0, 0.0), (75.0, 0.0)]
                        elif batch.direction == "vertical":
                            obs_width_mm = 150.0
                            obs_height_mm = 50.0
                            offsets = [(0.0, -75.0), (0.0, -25.0), (0.0, 25.0), (0.0, 75.0)]
                        else:
                            # Skip if no direction
                            self.node.get_logger().warn(
                                f"⚠️  Batch '{batch_known_id}' has no direction, skipping obstacle creation"
                            )
                            continue
                        
                        # Create 4 obstacles for this batch
                        for idx, (offset_x, offset_y) in enumerate(offsets):
                            obs = Obstacle()
                            obs.id = f"{source}_{batch_known_id}_member{idx}"
                            obs.type = 'crate'
                            obs.color = pattern_colors[idx] if idx < len(pattern_colors) else pattern_colors[-1]
                            obs.width_mm = obs_width_mm
                            obs.height_mm = obs_height_mm
                            
                            # Calculate obstacle center using predefined offsets
                            obs_center_x = batch_center_x_mm + offset_x
                            obs_center_y = batch_center_y_mm + offset_y
                            
                            # Convert to TOP-LEFT corner
                            obs.pose.x = obs_center_x - (obs_width_mm / 2.0)
                            obs.pose.y = obs_center_y - (obs_height_mm / 2.0)
                            obs.pose.theta = 0.0
                            
                            obs.detected_at = msg.stamp
                            
                            # Add to obstacles list and track source
                            self.obstacles.append(obs)
                            self.obstacle_sources[obs.id] = source
                        
                        # Mark this batch as having obstacles created
                        with self.batches_lock:
                            self.batches_with_obstacles.add(batch_known_id)
                
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

                        # Determine obstacle dimensions based on batch direction
                        if batch.direction == "horizontal":
                            obs.width_mm = self.params['obstacles']['crate_horizontal']['height_mm']
                            obs.height_mm = self.params['obstacles']['crate_horizontal']['width_mm']
                        elif batch.direction == "vertical":
                            obs.width_mm = self.params['obstacles']['crate_vertical']['height_mm']
                            obs.height_mm = self.params['obstacles']['crate_vertical']['width_mm']
                        else:
                            # Skip if no valid direction
                            self.node.get_logger().warn(
                                f"⚠️  Obstacle '{obs.id}' has no direction, skipping"
                            )
                            continue

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
                    
                    # Mark this batch as having obstacles created
                    with self.batches_lock:
                        self.batches_with_obstacles.add(batch.known_id)
            
            # Obstacle counts are logged in final summary

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
    
    def delete_batch(self, known_id: str) -> bool:
        """
        Delete a batch by its known_id from both FX and SX storage
        
        Args:
            known_id: The batch known_id to delete (e.g., 'f1', 's1', 'g2')
        
        Returns:
            True if batch was found and deleted, False otherwise
        """
        with self.batches_lock:
            deleted_from_fx = False
            deleted_from_sx = False
            
            if known_id in self.batches_fx:
                del self.batches_fx[known_id]
                deleted_from_fx = True
                self.node.get_logger().info(f"🗑️  Deleted batch '{known_id}' from FX storage")
            
            if known_id in self.batches_sx:
                del self.batches_sx[known_id]
                deleted_from_sx = True
                self.node.get_logger().info(f"🗑️  Deleted batch '{known_id}' from SX storage")
            
            # Remove from obstacle creation tracking
            self.batches_with_obstacles.discard(known_id)
            
            if not (deleted_from_fx or deleted_from_sx):
                self.node.get_logger().warn(f"⚠️  Batch '{known_id}' not found in FX or SX storage")
                return False
        
        # Remove all obstacles associated with this batch
        with self.obstacles_lock:
            # Find obstacles that belong to this batch (format: fx_{known_id}_member{idx} or sx_{known_id}_member{idx})
            obstacles_to_remove = []
            for obs in self.obstacles:
                # Check if obstacle ID contains the batch known_id
                if f"fx_{known_id}_" in obs.id or f"sx_{known_id}_" in obs.id:
                    obstacles_to_remove.append(obs)
            
            # Remove found obstacles
            removed_count = 0
            for obs in obstacles_to_remove:
                self.obstacles.remove(obs)
                # Remove from source tracking
                if obs.id in self.obstacle_sources:
                    del self.obstacle_sources[obs.id]
                removed_count += 1
            
            if removed_count > 0:
                self.node.get_logger().info(f"🗑️  Removed {removed_count} obstacles for batch '{known_id}'")
        
        # Republish obstacles to update visualization
        self.publish_current_obstacles()
        
        return True
    
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
