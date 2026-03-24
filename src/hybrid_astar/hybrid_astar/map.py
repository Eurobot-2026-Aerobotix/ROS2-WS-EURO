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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# ROS2 messages
from std_msgs.msg import Empty, Int32
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray

# Custom messages
from hybrid_astar.msg import Obstacle, ObstacleArray
from aruco_mapper_msgs.msg import ArucoMapperState, Batch, BatchMember
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

        # Predefined FX put-down zone centers (in mm)
        self.putdown_fx_positions_mm = {
            'f1': {'x': 1200.0, 'y': 100.0},
            'f2': {'x': 1900.0, 'y': 700.0},
            'f3': {'x': 1200.0, 'y': 800.0},
            'f5': {'x': 1200.0, 'y': 1500.0},
            'f6': {'x': 1900.0, 'y': 1500.0},
            'f8': {'x': 1200.0, 'y': 2200.0},
            'f9': {'x': 1900.0, 'y': 2300.0},
            'f10': {'x': 1200.0, 'y': 2900.0},
        }
        self.putdown_zone_match_tolerance_mm = 400.0
        
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
        self.first_update_empty_retry_count = 0
        self.first_update_empty_retry_max = 6
        self.first_update_empty_reported = False
        self.first_update_start_logged = False
        
        # Flag to track if we've locked the baseline patterns
        self.baselines_locked = False
        self.locked_baseline_batch_ids: set = set()
        self.pending_baseline_lock_ids: set = set()

        # Only the first normal update_map flow is allowed to trigger baseline locking.
        self.baseline_lock_pending = False
        
        # Set to track known_ids that need to be retried (not in TRACKING state)
        self.known_ids_to_retry: set = set()
        
        # Set to track manually deleted batch known_ids (should not be re-added)
        self.deleted_batch_ids: set = set()
        
        # Set to track currently active batch known_ids (from last update)
        self.active_batch_ids: set = set()
        
        # Set to track batches that already have obstacles created (prevents duplicates)
        self.batches_with_obstacles: set = set()
        self.manual_putdown_batch_ids: set = set()

        # Retry management for auto update_map
        self.retry_timer = None
        self.retry_timer_active = False
        self.retry_interval_sec = params.get('rates', {}).get('update_map_retry_sec', 1.0)
        self.retry_pending = 0
        self.retry_client = None
        self.retry_service_name = ""

        # Short-pattern retry: re-check batches whose locked pattern has < 4 elements
        self.short_pattern_ids: set = set()
        self.short_pattern_retry_timer = None
        self.short_pattern_retry_active = False
        self.short_pattern_retry_interval_sec = params.get('rates', {}).get('short_pattern_retry_sec', 2.0)

        # Enemy obstacle freshness management (/obstacle_relative)
        self.enemy_obstacle_timeout_sec = params.get('rates', {}).get('enemy_obstacle_timeout_sec', 1.0)
        self.last_enemy_update_ns: Optional[int] = None
        self.enemy_from_web: bool = False
        
        # Create publishers
        self.current_obs_pub = node.create_publisher(
            ObstacleArray, '/obstacles/current', 10)

        # Dedicated stream for active batch obstacles (sx_/fx_ only)
        self.path_viz_pub = node.create_publisher(
            MarkerArray, '/path_visualization', 10)
        
        self.relative_obstacle_marker_pub = node.create_publisher(
            MarkerArray, '/relative_obstacle_marker', 10)

        beacon_qos = QoSProfile(depth=1)
        beacon_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        beacon_qos.reliability = ReliabilityPolicy.RELIABLE
        self.beacon_status_pub = node.create_publisher(Int32, '/web/beacon_status', beacon_qos)
        self.beacon_ready_published = False
        
        # Create subscribers
        node.create_subscription(
            Empty, '/obstacles/clear_all', self.handle_clear_all, 10)
        
        node.create_subscription(
            PoseStamped, '/obstacle_relative', self.handle_obstacle_relative, 10)
        
        # Create timer for publishing current obstacles
        obstacle_hz = params['rates']['obstacle_list_hz']
        node.create_timer(1.0 / obstacle_hz, self.publish_current_obstacles)

        # Create persistent timers once and gate execution with active flags.
        # This avoids cancel() races with MultiThreadedExecutor that can crash with
        # "failed to call timer: timer is canceled".
        retry_interval = max(self.retry_interval_sec, 0.1)
        self.retry_timer = self.node.create_timer(retry_interval, self._retry_timer_callback)

        short_retry_interval = max(self.short_pattern_retry_interval_sec, 0.5)
        self.short_pattern_retry_timer = self.node.create_timer(
            short_retry_interval, self._short_pattern_retry_callback
        )
        
        self.node.get_logger().info('🗺️  Map Manager initialized')

    def _publish_beacon_status(self, value: int) -> None:
        msg = Int32()
        msg.data = int(value)
        self.beacon_status_pub.publish(msg)
        self.node.get_logger().info(f"📡 Published beacon status from map manager: {msg.data}")
    
    def handle_clear_all(self, msg: Empty):
        """Clear all obstacles"""
        with self.obstacles_lock:
            count = len(self.obstacles)
            self.obstacles.clear()
            self.obstacle_sources.clear()
            self.last_enemy_update_ns = None
            self.enemy_from_web = False
            self.node.get_logger().info(f"🗑️  Cleared all {count} obstacles")
        
        # Clear batch obstacle tracking
        with self.batches_lock:
            self.batches_with_obstacles.clear()
            self.manual_putdown_batch_ids.clear()
        
        # Publish obstacle visualization after clearing
        self.publish_obstacles(self.get_active_obstacles())
    
    def handle_clear_map(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Service handler: Clear all obstacles and reset map state for fresh update"""
        self.node.get_logger().info("🗺️  Clear map service called")

        # Prevent clear/reset operations from triggering baseline lock via retry-stop path.
        self.baseline_lock_pending = False
        
        # Stop any running retry timer
        self._stop_retry_timer()
        
        # Clear all obstacles
        with self.obstacles_lock:
            obstacle_count = len(self.obstacles)
            self.obstacles.clear()
            self.obstacle_sources.clear()
            self.last_enemy_update_ns = None
            self.enemy_from_web = False
        
        # Clear all batches
        with self.batches_lock:
            fx_count = len(self.batches_fx)
            sx_count = len(self.batches_sx)
            self.batches_fx.clear()
            self.batches_sx.clear()
            self.batches_with_obstacles.clear()  # Clear tracking
            self.manual_putdown_batch_ids.clear()
        
        # Reset state flags
        self.first_update_map = True
        self.first_update_empty_retry_count = 0
        self.baselines_locked = False
        self.beacon_ready_published = False
        self.locked_baseline_batch_ids.clear()
        self.pending_baseline_lock_ids.clear()
        self.baseline_lock_pending = False
        self.known_ids_to_retry.clear()
        self.deleted_batch_ids.clear()
        self.active_batch_ids.clear()
        self.short_pattern_ids.clear()
        self._stop_short_pattern_retry_timer()
        
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

    def _find_putdown_fx_known_id(self, batch_center_x_mm: float, batch_center_y_mm: float) -> Optional[str]:
        """Match a put-down batch center to the nearest predefined FX zone known_id."""
        best_known_id = None
        best_distance = float('inf')

        for known_id, pos in self.putdown_fx_positions_mm.items():
            dx = batch_center_x_mm - pos['x']
            dy = batch_center_y_mm - pos['y']
            distance = math.sqrt(dx * dx + dy * dy)
            if distance < best_distance:
                best_distance = distance
                best_known_id = known_id

        if best_known_id is None or best_distance > self.putdown_zone_match_tolerance_mm:
            self.node.get_logger().warn(
                f"⚠️  Put-down batch center ({batch_center_x_mm:.1f}, {batch_center_y_mm:.1f}) "
                f"did not match any FX zone within {self.putdown_zone_match_tolerance_mm:.0f}mm "
                f"(best={best_known_id}, dist={best_distance:.1f}mm)"
            )
            return None

        self.node.get_logger().info(
            f"📍 Put-down batch center matched FX zone '{best_known_id}' "
            f"(dist={best_distance:.1f}mm)"
        )
        return best_known_id

    def register_put_down_batch(
        self,
        batch_center_x_mm: float,
        batch_center_y_mm: float,
        robot_x_mm: float,
        robot_y_mm: float,
        robot_theta: float,
        color: str,
        stamp=None,
    ) -> tuple[bool, str, list[str], Optional[str]]:
        """
        Register a manual put-down batch as a tracked FX batch.

        The batch is assigned to the nearest predefined FX zone (`f1`, `f2`, ...).
        On later `update_map` calls, that FX known_id is preserved from the manually
        placed pose while the beacon still reports it. If the beacon loses it, the
        manual batch is removed.
        """
        known_id = self._find_putdown_fx_known_id(batch_center_x_mm, batch_center_y_mm)
        if known_id is None:
            return False, "Could not match put_down position to any FX known_id", [], None

        if stamp is None:
            stamp = self.node.get_clock().now().to_msg()

        crate_length_mm = 150.0
        crate_width_mm = 50.0
        gap_between_crates_mm = 20.0
        row_step_mm = crate_length_mm + gap_between_crates_mm
        lateral_step_mm = crate_width_mm + gap_between_crates_mm
        lateral_offsets = [
            -1.5 * lateral_step_mm,
            -0.5 * lateral_step_mm,
            0.5 * lateral_step_mm,
            1.5 * lateral_step_mm,
        ]

        ux = math.cos(robot_theta)
        uy = math.sin(robot_theta)
        vx = -math.sin(robot_theta)
        vy = math.cos(robot_theta)

        with self.batches_lock:
            existing_batch = self.batches_fx.get(known_id)
            existing_is_manual = known_id in self.manual_putdown_batch_ids

        existing_members = list(existing_batch.members) if (existing_is_manual and existing_batch is not None) else []
        existing_count = len(existing_members)
        if existing_count >= 8:
            return False, f"Batch {known_id} already contains 8 manually put-down members", [], known_id

        member_positions = []
        for lat in lateral_offsets:
            member_positions.append((
                batch_center_x_mm + lat * vx,
                batch_center_y_mm + lat * vy,
            ))

        all_members = existing_members[:]
        created_ids = []
        start_idx = len(all_members)
        for idx, (member_x_mm, member_y_mm) in enumerate(member_positions, start=start_idx):
            member = BatchMember()
            member.aruco_id = idx
            member.color = color
            member.map_pos = Point(
                x=member_x_mm / 1000.0,
                y=member_y_mm / 1000.0,
                z=0.0,
            )
            all_members.append(member)

        spread_dx = all_members[-1].map_pos.x - all_members[0].map_pos.x if len(all_members) > 1 else 0.0
        spread_dy = all_members[-1].map_pos.y - all_members[0].map_pos.y if len(all_members) > 1 else 0.0
        direction = 'horizontal' if abs(spread_dx) >= abs(spread_dy) else 'vertical'

        batch = Batch()
        batch.batch_id = 0
        batch.known_id = known_id
        batch.direction = direction
        batch.count = len(all_members)
        batch.state = 'TRACKING'
        existing_pattern = existing_batch.pattern if (existing_is_manual and existing_batch is not None and existing_batch.pattern) else ''
        batch.pattern = existing_pattern + ('yyyy' if color == 'yellow' else 'bbbb')

        avg_x_mm = sum(member.map_pos.x for member in all_members) * 1000.0 / len(all_members)
        avg_y_mm = sum(member.map_pos.y for member in all_members) * 1000.0 / len(all_members)
        batch.center = Point(
            x=avg_x_mm / 1000.0,
            y=avg_y_mm / 1000.0,
            z=0.0,
        )
        batch.members.extend(all_members)

        with self.obstacles_lock:
            if existing_count == 0:
                self.obstacles = [
                    obs for obs in self.obstacles
                    if not (obs.id.startswith(f"fx_{known_id}_") or obs.id.startswith(f"sx_{known_id}_"))
                ]
                self.obstacle_sources = {
                    k: v for k, v in self.obstacle_sources.items()
                    if not (k.startswith(f"fx_{known_id}_") or k.startswith(f"sx_{known_id}_"))
                }

            for idx, member in enumerate(all_members[start_idx:], start=start_idx):
                member_x_mm = member.map_pos.x * 1000.0
                member_y_mm = member.map_pos.y * 1000.0
                obs = Obstacle()
                obs.id = f"fx_{known_id}_member{idx}"
                obs.type = 'crate'
                obs.color = member.color
                obs.width_mm = crate_length_mm
                obs.height_mm = crate_width_mm
                obs.pose.x = member_x_mm
                obs.pose.y = member_y_mm
                obs.pose.theta = robot_theta
                obs.detected_at = stamp
                self.obstacles.append(obs)
                self.obstacle_sources[obs.id] = 'put_down'
                created_ids.append(obs.id)

        with self.batches_lock:
            self.batches_fx[known_id] = batch
            self.batches_with_obstacles.add(known_id)
            self.manual_putdown_batch_ids.add(known_id)
            self.active_batch_ids.add(known_id)

        self.publish_obstacles(self.get_active_obstacles())
        if existing_count > 0:
            return True, f"Appended put_down to {known_id} ({existing_count} -> {len(all_members)} members)", created_ids, known_id
        return True, f"Registered put_down batch as {known_id}", created_ids, known_id

    def _reconcile_manual_putdown_batches(self, received_batch_ids: set, lost_batch_ids: set) -> set:
        """
        Keep manual put-down FX batches unless the beacon explicitly reports
        that known_id with state LOST.

        Returns:
            Set of manual put-down known_ids that should be preserved this cycle.
        """
        with self.batches_lock:
            manual_ids = set(self.manual_putdown_batch_ids)

        # Preserve manual batches by default, and only remove if explicitly LOST.
        retained_manual_ids = set(manual_ids)
        lost_manual_ids = manual_ids & lost_batch_ids

        if lost_manual_ids:
            with self.obstacles_lock:
                self.obstacles = [
                    obs for obs in self.obstacles
                    if not any(obs.id.startswith(f"fx_{bid}_") for bid in lost_manual_ids)
                ]
                self.obstacle_sources = {
                    k: v for k, v in self.obstacle_sources.items()
                    if not any(k.startswith(f"fx_{bid}_") for bid in lost_manual_ids)
                }

            with self.batches_lock:
                for bid in lost_manual_ids:
                    self.batches_fx.pop(bid, None)
                    self.batches_with_obstacles.discard(bid)
                    self.manual_putdown_batch_ids.discard(bid)
                    self.active_batch_ids.discard(bid)
                    retained_manual_ids.discard(bid)

            self.node.get_logger().info(
                f"🗑️  Removed {len(lost_manual_ids)} manual put_down batches reported LOST by beacon: {lost_manual_ids}"
            )

        if retained_manual_ids:
            self.node.get_logger().info(
                f"📌 Preserving manual put_down batches from beacon refresh: {retained_manual_ids}"
            )

        return retained_manual_ids
    
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
            
            # Store center position (center convention used throughout)
            obs.pose.x = center_x_mm
            obs.pose.y = center_y_mm
            obs.pose.theta = self.quaternion_to_yaw(msg.pose.orientation)
            
            obs.detected_at = msg.header.stamp
            
            # Add to obstacles list and track source
            self.obstacles.append(obs)
            self.obstacle_sources[obs.id] = 'enemy'
            self.last_enemy_update_ns = self.node.get_clock().now().nanoseconds
            self.enemy_from_web = (msg.header.frame_id == 'map_web_enemy')
        
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
        self._purge_stale_enemy_obstacle()

        with self.obstacles_lock:
            msg = ObstacleArray()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.obstacles = list(self.obstacles)
            self.current_obs_pub.publish(msg)

        # Also publish obstacle visualization for RViz
        self.publish_obstacles(self.get_active_obstacles())

    def _purge_stale_enemy_obstacle(self):
        """Remove enemy obstacle if /obstacle_relative has not been updated recently."""
        # Keep web-provided enemy obstacle persistent until replaced/cleared
        if self.enemy_from_web:
            return

        if self.last_enemy_update_ns is None:
            return

        now_ns = self.node.get_clock().now().nanoseconds
        age_sec = (now_ns - self.last_enemy_update_ns) / 1e9
        if age_sec <= self.enemy_obstacle_timeout_sec:
            return

        with self.obstacles_lock:
            had_enemy = any(obs.color == 'enemy' for obs in self.obstacles)
            if had_enemy:
                self.obstacles = [obs for obs in self.obstacles if obs.color != 'enemy']
                self.obstacle_sources = {k: v for k, v in self.obstacle_sources.items() if v != 'enemy'}
                self.node.get_logger().info(
                    f"⏱️ Removed stale enemy obstacle (age={age_sec:.2f}s > timeout={self.enemy_obstacle_timeout_sec:.2f}s)"
                )

        # Reset freshness marker after stale removal
        self.last_enemy_update_ns = None
        self.enemy_from_web = False
    
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
            
            # Position (on ground level) - x/y are centers
            crate_marker.pose.position.x = obs['x'] / 1000.0
            crate_marker.pose.position.y = obs['y'] / 1000.0
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
            if not self.first_update_start_logged:
                self.node.get_logger().info("🔄 First update_map call - requesting all batches (unlocked)")
                self.first_update_start_logged = True
            self.baseline_lock_pending = True
        
        stable_request = GetStableBatches.Request()
        stable_request.batch_types_filter = "sfg"  # Get all stable batches
        stable_request.specific_known_id = ""  # Get all batches matching the filter
        stable_request.only_locked_baselines = False  # Always False for update_map - lock call happens after all TRACKING
        
        # Call service asynchronously
        future = client_to_use.call_async(stable_request)
        future.add_done_callback(lambda f: self._handle_get_stable_batches_response(
            f, response, client_to_use, service_name, None, False, True))
        
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

        # Rearm initial update flow so the next /update_map call behaves like a fresh first update.
        # reset_map is a reset/reinitialization step, not a normal incremental update.
        self.first_update_map = True
        self.first_update_empty_retry_count = 0
        self.baselines_locked = False
        self.baseline_lock_pending = False
        self.beacon_ready_published = False
        self.locked_baseline_batch_ids.clear()
        self.pending_baseline_lock_ids.clear()
        self.known_ids_to_retry.clear()
        self.short_pattern_ids.clear()
        self._stop_retry_timer()
        self._stop_short_pattern_retry_timer()

        self.node.get_logger().info("🔄 reset_map called - requesting batches with 'rrr' filter")

        stable_request = GetStableBatches.Request()
        stable_request.batch_types_filter = "rrr"
        stable_request.specific_known_id = ""
        stable_request.only_locked_baselines = False

        future = client_to_use.call_async(stable_request)
        future.add_done_callback(lambda f: self._handle_get_stable_batches_response(
            f, response, client_to_use, service_name, None, False, False))

        response.success = True
        response.message = f"Reset map triggered from {service_name} with 'rrr' filter (processing asynchronously)"
        return response

    def _handle_get_stable_batches_response(self, future, trigger_response=None, client=None, service_name="", specific_known_id=None, from_retry_timer=False, can_lock_baseline=True):
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
            lost_batch_ids = {
                batch.known_id
                for batch in aruco_state.batches
                if batch.state.upper() == 'LOST'
            }
            
            # Initialize variables that may be used in final summary
            disappeared_ids = set()
            
            # Remember if this is the first update (before we set it to False)
            is_first_update = can_lock_baseline and self.first_update_map and specific_known_id is None
            is_full_update = specific_known_id is None  # Full update (not a specific retry)

            # If first update returns zero batches, retry as first update (do not switch to incremental flow yet).
            if is_first_update and num_batches == 0:
                if self.first_update_empty_retry_count < self.first_update_empty_retry_max:
                    self.first_update_empty_retry_count += 1
                    attempt = self.first_update_empty_retry_count
                    self.baseline_lock_pending = True
                    retry_msg = (
                        f"⚠️ First update_map returned 0 batches, retrying as first update "
                        f"({attempt}/{self.first_update_empty_retry_max})"
                    )
                    if attempt == 1:
                        self.node.get_logger().warn(retry_msg)
                    else:
                        self.node.get_logger().debug(retry_msg)

                    retry_client = client if client is not None else self.retry_client
                    if retry_client and retry_client.service_is_ready():
                        retry_request = GetStableBatches.Request()
                        retry_request.batch_types_filter = "sfg"
                        retry_request.specific_known_id = ""
                        retry_request.only_locked_baselines = False

                        retry_future = retry_client.call_async(retry_request)
                        retry_future.add_done_callback(
                            lambda f: self._handle_get_stable_batches_response(
                                f, trigger_response, retry_client, service_name, None, False, True
                            )
                        )
                    else:
                        self.node.get_logger().error(
                            "❌ Cannot retry first update_map: batch service not ready"
                        )
                    return

                exhausted_msg = (
                    f"❌ First update_map still returned 0 batches after "
                    f"{self.first_update_empty_retry_max} retries"
                )
                if not self.first_update_empty_reported:
                    self.node.get_logger().error(exhausted_msg)
                    self.first_update_empty_reported = True
                else:
                    self.node.get_logger().debug(exhausted_msg)
                return

            if is_first_update and num_batches > 0:
                self.first_update_empty_reported = False
                self.first_update_start_logged = False
            
            # Filter batches based on update type
            if is_first_update:
                # FIRST UPDATE: accept non-EMERGING batches, queue EMERGING SX for retry
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
                        # Only retry EMERGING SX batches
                        if batch.known_id.startswith('s'):
                            emerging_ids.add(batch.known_id)
                            self.node.get_logger().info(f"  ⏳ [SX] '{batch.known_id}': {state} → will retry")
                        else:
                            skipped_count += 1
                
                # Store EMERGING IDs for retry (only SX and GX)
                self.known_ids_to_retry = emerging_ids
                
                self.node.get_logger().info(
                    f"📊 First update summary: {non_emerging_count} non-EMERGING, {len(emerging_ids)} SX EMERGING queued for retry, "
                    f"{skipped_count} FX skipped"
                )

                # First update completed with non-empty data; clear empty-response retry counter.
                self.first_update_empty_retry_count = 0
                
                # Mark first update as complete
                self.first_update_map = False

                # Start auto-retry timer if needed
                if self.known_ids_to_retry:
                    self._start_retry_timer()
                    
            elif is_full_update:
                # SUBSEQUENT FULL UPDATE: process non-EMERGING batches, retry EMERGING SX only
                filtered_batches = []
                emerging_ids = set()
                non_emerging_count = 0
                skipped_count = 0
                
                for batch in aruco_state.batches:
                    state = batch.state.upper()
                    is_fx = batch.known_id.startswith('f') or not (batch.known_id.startswith('s') or batch.known_id.startswith('g'))
                    is_sx = batch.known_id.startswith('s')
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
                        elif is_sx:
                            # EMERGING SX: Add to retry queue (log only on first add)
                            if batch.known_id not in emerging_ids and batch.known_id not in self.known_ids_to_retry:
                                self.node.get_logger().info(f"  ⏭️  [{batch_type}] '{batch.known_id}': EMERGING → added to retry")
                            emerging_ids.add(batch.known_id)
                            skipped_count += 1
                        else:
                            # EMERGING FX/GX - skip
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
                self.node.get_logger().debug(f"✅ After deduplication: {len(filtered_batches)} unique batches")
            
            # Get known_ids from current response
            received_batch_ids = {batch.known_id for batch in filtered_batches}
            retained_manual_putdown_ids = set()
            preserved_manual_batches = {}

            if specific_known_id is None:
                retained_manual_putdown_ids = self._reconcile_manual_putdown_batches(
                    received_batch_ids,
                    lost_batch_ids
                )
                if retained_manual_putdown_ids:
                    with self.batches_lock:
                        preserved_manual_batches = {
                            bid: self.batches_fx[bid]
                            for bid in retained_manual_putdown_ids
                            if bid in self.batches_fx
                        }
            
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
                            self.locked_baseline_batch_ids.clear()
                            self.pending_baseline_lock_ids.clear()
                            self.batches_fx.update(preserved_manual_batches)
                            self.batches_with_obstacles.update(preserved_manual_batches.keys())
                            self.manual_putdown_batch_ids.update(preserved_manual_batches.keys())
                            self.node.get_logger().info(f"🗑️  Cleared all FX and SX batches and {removed} obstacles before first update")
                else:
                    # SUBSEQUENT UPDATES: Incremental update
                    # 1. Find batches that disappeared (in active but not in received)
                    disappeared_ids = self.active_batch_ids - received_batch_ids
                    # Preserved manual put_down batches must not be considered disappeared.
                    disappeared_ids -= retained_manual_putdown_ids
                    
                    # 2. Remove obstacles ONLY for batches that disappeared OR are FX (FX always refreshed)
                    # Keep existing obstacles for SX batches that are still present
                    fx_batches_ids = {batch.known_id for batch in filtered_batches if batch.known_id.startswith('f')}
                    fx_batches_ids -= retained_manual_putdown_ids
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
                                    self.locked_baseline_batch_ids.discard(bid)
                                    self.pending_baseline_lock_ids.discard(bid)
                                    self.short_pattern_ids.discard(bid)
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
                                self.node.get_logger().debug(
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

                    # 3b. Keep manual put_down batches at their manually registered pose.
                    if retained_manual_putdown_ids:
                        pre_filter_count = len(filtered_batches)
                        filtered_batches = [b for b in filtered_batches if b.known_id not in retained_manual_putdown_ids]
                        skipped_count = pre_filter_count - len(filtered_batches)
                        if skipped_count > 0:
                            self.node.get_logger().info(
                                f"📌 Skipping {skipped_count} beacon FX batches because manual put_down pose is preserved: "
                                f"{retained_manual_putdown_ids}"
                            )
                    
                    # 4. Filter out already-tracked SX/GX batches (keep existing obstacles, only add new ones)
                    # FX batches are always processed (refreshed every update)
                    new_batches = [b for b in filtered_batches 
                                   if b.known_id not in self.active_batch_ids or b.known_id.startswith('f')]
                    if len(new_batches) < len(filtered_batches):
                        kept_count = len(filtered_batches) - len(new_batches)
                        self.node.get_logger().debug(
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

            # Never re-add a beacon FX batch if a manual put_down batch currently owns that known_id.
            if specific_known_id is None and retained_manual_putdown_ids:
                pre_filter_count = len(filtered_batches)
                filtered_batches = [b for b in filtered_batches if b.known_id not in retained_manual_putdown_ids]
                skipped_count = pre_filter_count - len(filtered_batches)
                if skipped_count > 0:
                    self.node.get_logger().info(
                        f"📌 Preserving {skipped_count} manual put_down batches at their registered pose: "
                        f"{retained_manual_putdown_ids}"
                    )
            
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
                self.active_batch_ids = received_batch_ids | retained_manual_putdown_ids
                
                # Check if all batches are TRACKING and we haven't locked baselines yet
                if can_lock_baseline and self.baseline_lock_pending and not self.baselines_locked and not self.known_ids_to_retry:
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
                    self.node.get_logger().debug(
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
        if self.retry_timer_active:
            return

        self.retry_timer_active = True
        interval = max(self.retry_interval_sec, 0.1)
        self.node.get_logger().info(
            f"⏳ Auto-retry enabled: checking EMERGING batches every {interval:.1f}s until stable"
        )

    def _stop_retry_timer(self):
        """Stop periodic retries when all batches are TRACKING or removed."""
        if not self.retry_timer_active:
            return

        self.retry_timer_active = False
        self.retry_pending = 0
        self.node.get_logger().info("✅ Auto-retry stopped: all batches are no longer EMERGING")
        
        # After all batches are no longer EMERGING, lock the baseline patterns
        if self.baseline_lock_pending and not self.baselines_locked:
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

    def _request_locked_baseline_for_sx(self, known_id: str):
        """Request locked baseline for a single SX batch when it appears after initial lock."""
        if not known_id.startswith('s'):
            return
        if known_id in self.locked_baseline_batch_ids or known_id in self.pending_baseline_lock_ids:
            return

        client = None
        if self.retry_client and self.retry_client.service_is_ready():
            client = self.retry_client
        elif self.get_stable_batches_client.service_is_ready():
            client = self.get_stable_batches_client
            self.retry_client = client
            self.retry_service_name = "/aruco_mapper/get_stable_batches"
        elif self.get_stable_batches_client_web and self.get_stable_batches_client_web.service_is_ready():
            client = self.get_stable_batches_client_web
            self.retry_client = client
            self.retry_service_name = "/aruco_mapper/get_stable_batches_web"
        else:
            self.node.get_logger().warn(
                f"⚠️ Cannot request locked baseline for '{known_id}': batch service not ready"
            )
            return

        self.pending_baseline_lock_ids.add(known_id)
        self.node.get_logger().info(
            f"🔒 Requesting locked baseline for SX batch '{known_id}'"
        )

        req = GetStableBatches.Request()
        req.batch_types_filter = "sfg"
        req.specific_known_id = known_id
        req.only_locked_baselines = True
        future = client.call_async(req)
        future.add_done_callback(
            lambda f, kid=known_id: self._handle_single_locked_sx_response(f, kid)
        )

    def _handle_single_locked_sx_response(self, future, known_id: str):
        """Handle locked-baseline response for a single SX batch."""
        try:
            response = future.result()
            if not response or not response.stable_state:
                self.node.get_logger().warn(
                    f"⚠️ Locked baseline request for '{known_id}' returned no response"
                )
                return

            target_batch = None
            for batch in response.stable_state.batches:
                if batch.known_id == known_id:
                    target_batch = batch
                    break

            if target_batch is None:
                self.node.get_logger().info(
                    f"⏳ Locked baseline not available yet for '{known_id}'"
                )
                return

            pattern = target_batch.pattern if target_batch.pattern else ""
            self.locked_baseline_batch_ids.add(known_id)
            with self.batches_lock:
                self.batches_sx[known_id] = target_batch
                self.batches_with_obstacles.discard(known_id)

            self.node.get_logger().info(
                f"✅ Locked baseline received for SX '{known_id}' pattern='{pattern}'"
            )

            if len(pattern) < 4:
                self.short_pattern_ids.add(known_id)
                self.node.get_logger().warn(
                    f"⚠️  Locked pattern for '{known_id}' still short ({len(pattern)}), will re-check"
                )
                self._start_short_pattern_retry_timer()
                return

            member_count = len(target_batch.members) if target_batch.members else 0
            if member_count == 3:
                self.node.get_logger().info(
                    f"⏭️  SX batch '{known_id}' has only 3 members after baseline lock - skipping obstacle creation"
                )
                return

            prefix_sx = f"sx_{known_id}_"
            with self.obstacles_lock:
                self.obstacles = [obs for obs in self.obstacles if not obs.id.startswith(prefix_sx)]
                self.obstacle_sources = {
                    k: v for k, v in self.obstacle_sources.items() if not k.startswith(prefix_sx)
                }

            sx_state = ArucoMapperState()
            sx_state.stamp = response.stable_state.stamp
            sx_state.initialized = response.stable_state.initialized
            sx_state.batches = [target_batch]
            self._handle_aruco_mapper_state(sx_state, 'sx')
        except Exception as e:
            self.node.get_logger().error(
                f"❌ Error handling locked baseline for '{known_id}': {e}"
            )
            import traceback
            self.node.get_logger().error(traceback.format_exc())
        finally:
            self.pending_baseline_lock_ids.discard(known_id)
    
    def _handle_lock_baseline_response(self, future):
        """Handle response from locked baseline patterns request."""
        try:
            response = future.result()
            if response and response.stable_state:
                num_batches = len(response.stable_state.batches)
                self.baselines_locked = True
                self.baseline_lock_pending = False
                self.node.get_logger().info(f"✅ Baseline patterns locked! Received {num_batches} locked batches")

                if not self.beacon_ready_published:
                    self._publish_beacon_status(1)
                    self.beacon_ready_published = True
                
                # Track the locked batch IDs
                locked_batch_ids = set()
                sx_batches_ready_for_obstacles = []
                
                # Update batches with locked patterns
                with self.batches_lock:
                    for batch in response.stable_state.batches:
                        locked_batch_ids.add(batch.known_id)
                        if batch.known_id.startswith('s'):
                            self.locked_baseline_batch_ids.add(batch.known_id)
                            self.batches_sx[batch.known_id] = batch
                            self.node.get_logger().info(f"   🔒 Locked SX pattern: {batch.known_id} = '{batch.pattern}'")
                            if len(batch.pattern or "") >= 4:
                                sx_batches_ready_for_obstacles.append(batch)
                        elif batch.known_id.startswith('f'):
                            self.batches_fx[batch.known_id] = batch
                            self.node.get_logger().info(f"   🔒 Locked FX pattern: {batch.known_id} = '{batch.pattern}'")
                
                # CRITICAL: Add locked batches to active_batch_ids to prevent them from being removed
                self.active_batch_ids.update(locked_batch_ids)
                self.node.get_logger().info(f"   📌 Added {len(locked_batch_ids)} locked batches to active tracking")

                # Detect SX batches with short patterns (< 4 elements) and schedule re-checks
                for batch in response.stable_state.batches:
                    pattern = batch.pattern if batch.pattern else ""
                    if batch.known_id.startswith('s') and len(pattern) < 4:
                        self.short_pattern_ids.add(batch.known_id)
                        self.node.get_logger().warn(
                            f"⚠️  Short pattern after baseline lock for '{batch.known_id}': "
                            f"'{pattern}' ({len(pattern)} elements, expected 4) — will re-check"
                        )

                if self.short_pattern_ids:
                    self._start_short_pattern_retry_timer()

                if sx_batches_ready_for_obstacles:
                    with self.batches_lock:
                        for batch in sx_batches_ready_for_obstacles:
                            self.batches_with_obstacles.discard(batch.known_id)
                    with self.obstacles_lock:
                        ready_ids = {batch.known_id for batch in sx_batches_ready_for_obstacles}
                        self.obstacles = [
                            obs for obs in self.obstacles
                            if not any(obs.id.startswith(f"sx_{bid}_") for bid in ready_ids)
                        ]
                        self.obstacle_sources = {
                            k: v for k, v in self.obstacle_sources.items()
                            if not any(k.startswith(f"sx_{bid}_") for bid in ready_ids)
                        }
                    sx_state = ArucoMapperState()
                    sx_state.stamp = response.stable_state.stamp
                    sx_state.initialized = response.stable_state.initialized
                    sx_state.batches = sx_batches_ready_for_obstacles
                    self._handle_aruco_mapper_state(sx_state, 'sx')
            else:
                self.node.get_logger().error("❌ Failed to lock baseline patterns: no response")
        except Exception as e:
            self.node.get_logger().error(f"❌ Error locking baseline patterns: {e}")

    def _retry_timer_callback(self):
        """Timer callback to retry EMERGING SX known_ids only."""
        if not self.retry_timer_active:
            return

        if not self.known_ids_to_retry:
            self._stop_retry_timer()
            return

        if self.retry_pending > 0:
            return

        if not self.retry_client or not self.retry_client.service_is_ready():
            self.node.get_logger().warn("⚠️ Auto-retry: batch service not ready, will retry later")
            return

        self.node.get_logger().info(
            f"🔁 Auto-retrying {len(self.known_ids_to_retry)} EMERGING SX batches"
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
                    f, None, self.retry_client, self.retry_service_name, kid, True, True
                )
            )
    
    # ------------------------------------------------------------------
    # Short-pattern retry helpers
    # ------------------------------------------------------------------

    def _start_short_pattern_retry_timer(self):
        """Start a periodic timer that re-checks locked baselines for batches
        whose pattern currently has fewer than 4 colour elements."""
        if self.short_pattern_retry_active:
            return

        self.short_pattern_retry_active = True
        interval = max(self.short_pattern_retry_interval_sec, 0.5)
        self.node.get_logger().info(
            f"🔍 Short-pattern retry enabled: re-checking every {interval:.1f}s "
            f"for {len(self.short_pattern_ids)} batches: {self.short_pattern_ids}"
        )

    def _stop_short_pattern_retry_timer(self):
        """Stop the short-pattern retry timer."""
        if not self.short_pattern_retry_active:
            return

        self.short_pattern_retry_active = False
        self.node.get_logger().info("✅ Short-pattern retry stopped: all patterns resolved")

    def _short_pattern_retry_callback(self):
        """Timer callback: request locked baselines for each short-pattern batch
        to see whether the pattern has been extended to 4 elements."""
        if not self.short_pattern_retry_active:
            return

        if not self.short_pattern_ids:
            self._stop_short_pattern_retry_timer()
            return

        if not self.retry_client or not self.retry_client.service_is_ready():
            self.node.get_logger().warn("⚠️ Short-pattern retry: batch service not ready")
            return

        self.node.get_logger().info(
            f"🔍 Re-checking short-pattern batches with locked baseline: {self.short_pattern_ids}"
        )
        for known_id in list(self.short_pattern_ids):
            # Defensive: SX-only retry policy
            if not known_id.startswith('s'):
                self.short_pattern_ids.discard(known_id)
                continue

            req = GetStableBatches.Request()
            req.batch_types_filter = "sfg"
            req.specific_known_id = known_id
            req.only_locked_baselines = True
            future = self.retry_client.call_async(req)
            future.add_done_callback(
                lambda f, kid=known_id: self._handle_short_pattern_retry_response(f, kid)
            )

    def _handle_short_pattern_retry_response(self, future, known_id: str):
        """Handle the re-check response for a single short-pattern batch.
        If the pattern now has >= 4 elements, update the stored batch and
        regenerate its obstacles; remove it from the retry set."""
        try:
            response = future.result()
            if not response or not response.stable_state:
                self.node.get_logger().warn(
                    f"⚠️  Short-pattern re-check for '{known_id}': no response"
                )
                return

            for batch in response.stable_state.batches:
                if batch.known_id != known_id:
                    continue

                # Defensive: SX-only retry policy
                if not known_id.startswith('s'):
                    self.short_pattern_ids.discard(known_id)
                    break

                new_pattern = batch.pattern if batch.pattern else ""

                with self.batches_lock:
                    existing = self.batches_sx.get(known_id) or self.batches_fx.get(known_id)
                    old_pattern = existing.pattern if (existing and existing.pattern) else ""

                if len(new_pattern) >= 4:
                    self.node.get_logger().info(
                        f"✅ Short-pattern resolved for '{known_id}': "
                        f"'{old_pattern}' → '{new_pattern}' ({len(new_pattern)} elements)"
                    )

                    self.locked_baseline_batch_ids.add(known_id)

                    with self.batches_lock:
                        self.batches_sx[known_id] = batch
                        self.batches_with_obstacles.discard(known_id)

                    prefix_sx = f"sx_{known_id}_"
                    prefix_fx = f"fx_{known_id}_"
                    with self.obstacles_lock:
                        self.obstacles = [
                            obs for obs in self.obstacles
                            if not (obs.id.startswith(prefix_sx) or obs.id.startswith(prefix_fx))
                        ]
                        self.obstacle_sources = {
                            k: v for k, v in self.obstacle_sources.items()
                            if not (k.startswith(prefix_sx) or k.startswith(prefix_fx))
                        }

                    fake_state = ArucoMapperState()
                    fake_state.stamp = self.node.get_clock().now().to_msg()
                    fake_state.batches = [batch]
                    self._handle_aruco_mapper_state(fake_state, 'sx')

                    self.node.get_logger().info(
                        f"   🔄 Obstacles rebuilt for '{known_id}' with full pattern '{new_pattern}'"
                    )

                    self.short_pattern_ids.discard(known_id)
                    if not self.short_pattern_ids:
                        self._stop_short_pattern_retry_timer()

                elif len(new_pattern) > len(old_pattern):
                    self.node.get_logger().info(
                        f"📈 Pattern improving for '{known_id}': "
                        f"'{old_pattern}' → '{new_pattern}' (still {len(new_pattern)} elements, need 4)"
                    )
                else:
                    self.node.get_logger().info(
                        f"⏳ Pattern unchanged for '{known_id}': '{new_pattern}' ({len(new_pattern)} elements)"
                    )
                break
            else:
                self.node.get_logger().info(
                    f"⏳ Short-pattern re-check: no data yet for '{known_id}'"
                )
        except Exception as e:
            import traceback
            self.node.get_logger().error(f"❌ Short-pattern retry error for '{known_id}': {e}")
            self.node.get_logger().error(traceback.format_exc())

    # ------------------------------------------------------------------
    # ArUco mapper state processing
    # ------------------------------------------------------------------

    def _handle_aruco_mapper_state(self, msg: ArucoMapperState, source: str):
        """
        Internal method to handle ArUco mapper state updates
        source: 'fx', 'sx', or 'gx'
        Note: FX/SX batches and obstacles are cleared before this is called
        """
        with self.obstacles_lock:
            for batch in msg.batches:
                if source == 'gx':
                    continue

                if source == 'fx':
                    member_count = len(batch.members) if batch.members else 0
                    self.node.get_logger().debug(
                        f"📦 FX batch '{batch.known_id}' received with {member_count} members"
                    )
                    if not batch.members:
                        continue

                with self.batches_lock:
                    if source == 'fx':
                        self.batches_fx[batch.known_id] = batch
                    elif source == 'sx':
                        self.batches_sx[batch.known_id] = batch

                    if batch.known_id in self.batches_with_obstacles:
                        self.node.get_logger().info(
                            f"⏭️  Skipping '{batch.known_id}' - obstacles already created"
                        )
                        skip_creation = True
                    else:
                        skip_creation = False

                if skip_creation:
                    continue

                if source == 'sx':
                    batch_known_id = batch.known_id

                    if batch_known_id not in self.predefined_batch_positions:
                        self.node.get_logger().warn(
                            f"⚠️  Batch '{batch_known_id}' not found in predefined positions, skipping obstacle creation"
                        )
                        continue

                    batch_pos = self.predefined_batch_positions[batch_known_id]
                    batch.center.x = batch_pos["x"]
                    batch.center.y = batch_pos["y"]
                    batch.center.z = 0.0

                    if batch_known_id not in self.locked_baseline_batch_ids:
                        self._request_locked_baseline_for_sx(batch_known_id)
                        self.node.get_logger().info(
                            f"⏳ Deferring SX obstacles for '{batch_known_id}' until its baseline is locked"
                        )
                        continue

                    pattern_colors = self.parse_pattern_colors((batch.pattern or "").lower())
                    if len(pattern_colors) < 4:
                        self.node.get_logger().warn(
                            f"⚠️  Locked SX batch '{batch_known_id}' still has incomplete pattern '{batch.pattern}', waiting before obstacle creation"
                        )
                        continue

                    member_count = len(batch.members) if batch.members else 0
                    if member_count == 3:
                        self.node.get_logger().info(
                            f"⏭️  SX batch '{batch_known_id}' has only 3 members - skipping obstacle creation"
                        )
                        continue

                    batch_center_x_mm = batch_pos["x"] * 1000.0
                    batch_center_y_mm = batch_pos["y"] * 1000.0

                    if batch.direction == "horizontal":
                        obs_width_mm = 50.0
                        obs_height_mm = 150.0
                        offsets = [(-75.0, 0.0), (-25.0, 0.0), (25.0, 0.0), (75.0, 0.0)]
                    elif batch.direction == "vertical":
                        obs_width_mm = 150.0
                        obs_height_mm = 50.0
                        offsets = [(0.0, -75.0), (0.0, -25.0), (0.0, 25.0), (0.0, 75.0)]
                    else:
                        self.node.get_logger().warn(
                            f"⚠️  Batch '{batch_known_id}' has no direction, skipping obstacle creation"
                        )
                        continue

                    for idx, (offset_x, offset_y) in enumerate(offsets):
                        obs = Obstacle()
                        obs.id = f"{source}_{batch_known_id}_member{idx}"
                        obs.type = 'crate'
                        obs.color = pattern_colors[idx]
                        obs.width_mm = obs_width_mm
                        obs.height_mm = obs_height_mm
                        obs_center_x = batch_center_x_mm + offset_x
                        obs_center_y = batch_center_y_mm + offset_y
                        obs.pose.x = obs_center_x  # center convention
                        obs.pose.y = obs_center_y  # center convention
                        obs.pose.theta = 0.0
                        obs.detected_at = msg.stamp
                        self.obstacles.append(obs)
                        self.obstacle_sources[obs.id] = source

                    with self.batches_lock:
                        self.batches_with_obstacles.add(batch_known_id)
                    continue

                pattern_str = batch.pattern if batch.pattern else ""
                pattern_colors = self.parse_pattern_colors(pattern_str)

                for idx, member in enumerate(batch.members):
                    obs = Obstacle()
                    obs.id = f"{source}_{batch.known_id}_member{idx}"
                    obs.type = 'crate'

                    if pattern_colors and idx < len(pattern_colors):
                        obs.color = pattern_colors[idx]
                    elif member.color in ["yellow", "blue"]:
                        obs.color = member.color
                    else:
                        continue

                    if batch.direction == "horizontal":
                        obs.width_mm = self.params['obstacles']['crate_horizontal']['height_mm']
                        obs.height_mm = self.params['obstacles']['crate_horizontal']['width_mm']
                    elif batch.direction == "vertical":
                        obs.width_mm = self.params['obstacles']['crate_vertical']['height_mm']
                        obs.height_mm = self.params['obstacles']['crate_vertical']['width_mm']
                    else:
                        self.node.get_logger().warn(
                            f"⚠️  Obstacle '{obs.id}' has no direction, skipping"
                        )
                        continue

                    center_x_mm = member.map_pos.x * 1000.0
                    center_y_mm = member.map_pos.y * 1000.0
                    obs.pose.x = center_x_mm  # center convention
                    obs.pose.y = center_y_mm  # center convention
                    obs.pose.theta = 0.0
                    obs.detected_at = msg.stamp

                    self.obstacles.append(obs)
                    self.obstacle_sources[obs.id] = source

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
            self.locked_baseline_batch_ids.discard(known_id)
            self.pending_baseline_lock_ids.discard(known_id)
            self.short_pattern_ids.discard(known_id)
            self.manual_putdown_batch_ids.discard(known_id)
            
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
    
    def get_active_batches_info(self) -> Dict[str, list]:
        """
        Get information about all active batches
        
        Returns:
            Dictionary with arrays for each field:
            - known_ids: List of batch known_ids (e.g., 's1', 'f_batch_1')
            - batch_types: List of batch types ('fx', 'sx', 'gx')
            - member_counts: List of member counts
            - directions: List of directions ('horizontal', 'vertical', or '')
            - patterns: List of color patterns ('ybyb', etc.)
            - theta_degrees: List of batch orientations in degrees
            - centers: List of Point objects (batch centers)
            - qualities: List of quality indicators (all 1 for now)
        """
        with self.batches_lock:
            known_ids = []
            batch_types = []
            member_counts = []
            directions = []
            patterns = []
            theta_degrees = []
            centers = []
            qualities = []

            def _theta_from_direction(direction: str) -> float:
                if direction == 'horizontal':
                    return 0.0
                if direction == 'vertical':
                    return 90.0
                return 0.0
            
            # Process FX batches
            for known_id, batch in self.batches_fx.items():
                # Only include active batches (not manually deleted)
                if known_id not in self.deleted_batch_ids:
                    known_ids.append(known_id)
                    batch_types.append('fx')
                    member_counts.append(len(batch.members) if batch.members else 0)
                    direction = batch.direction if batch.direction else ''
                    directions.append(direction)
                    patterns.append(batch.pattern.lower() if getattr(batch, 'pattern', None) else '')
                    theta_degrees.append(_theta_from_direction(direction))
                    centers.append(batch.center)
                    qualities.append(1)  # Always 1 for now
            
            # Process SX batches
            for known_id, batch in self.batches_sx.items():
                # Only include active batches (not manually deleted)
                if known_id not in self.deleted_batch_ids:
                    known_ids.append(known_id)
                    batch_types.append('sx')
                    member_counts.append(len(batch.members) if batch.members else 0)
                    direction = batch.direction if batch.direction else ''
                    directions.append(direction)
                    patterns.append(batch.pattern.lower() if getattr(batch, 'pattern', None) else '')
                    theta_degrees.append(_theta_from_direction(direction))
                    centers.append(batch.center)
                    qualities.append(1)  # Always 1 for now
            
            return {
                'known_ids': known_ids,
                'batch_types': batch_types,
                'member_counts': member_counts,
                'directions': directions,
                'patterns': patterns,
                'theta_degrees': theta_degrees,
                'centers': centers,
                'qualities': qualities
            }
    
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
