#!/usr/bin/env python3
"""
Map Manager - Handles obstacle and batch management for path planning
Manages dynamic obstacles from ArUco mapper and enemy detection
"""

import math
import time
import json
from typing import Dict, List, Optional, Tuple
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# ROS2 messages
from std_msgs.msg import Empty, Int32, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray

# Custom messages
from hybrid_astar.msg import Obstacle, ObstacleArray
from hybrid_astar.srv import SetTeamColor
from aruco_mapper_msgs.msg import ArucoMapperState, Batch, BatchMember
from aruco_mapper_msgs.srv import GetStableBatches

# Global toggle: when True, batches reported LOST are removed.
# Set to False to preserve LOST batches.
ACTIVATE_LOST_BATCH_DELETING = True

# When True, periodically logs each batch's obstacle count and P/NP status.
DEBUG_BATCHES_MEMBERS = False


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
            'f4': {'x': 550.0, 'y': 1250.0},
            'f5': {'x': 1200.0, 'y': 1500.0},
            'f6': {'x': 1900.0, 'y': 1500.0},
            'f7': {'x': 550.0, 'y': 1750.0},
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
        # Optional FX/SX status type extracted from mapper state suffixes:
        #   TRACKING_P / EMERGING_P   -> 'P'
        #   TRACKING_NP / EMERGING_NP -> 'NP'
        self.batches_fx_status_type: Dict[str, str] = {}
        self.batches_sx_status_type: Dict[str, str] = {}
        # Optional SX locked-baseline status suffix:
        #   *_T -> tentative locked-baseline, keep retrying only_locked_baselines
        #   *_C -> confirmed locked-baseline, safe to finalize obstacles
        self.batches_sx_baseline_status_type: Dict[str, str] = {}
        self.batches_lock = Lock()
        
        # Static obstacles - using permanent area instead of JSON file
        self.static_obstacles: Dict[str, Obstacle] = {}
        
        # Flag for first crate update logging
        self.first_crate_update = False
        
        # Flag to track if this is the first update_map call
        self.first_update_map = True
        self.first_update_empty_retry_count = 0
        self.first_update_empty_retry_max = None
        self.first_update_empty_reported = False
        self.first_update_start_logged = False
        
        # Flag to track if we've locked the baseline patterns
        self.baselines_locked = False
        self.locked_baseline_batch_ids: set = set()
        self.locked_baseline_all_batch_ids: set = set()
        self.pending_baseline_lock_ids: set = set()

        # Only the first normal update_map flow is allowed to trigger baseline locking.
        self.baseline_lock_pending = False
        
        # Set to track known_ids that need to be retried (not in TRACKING state)
        self.known_ids_to_retry: set = set()
        
        # Set to track manually deleted batch known_ids (should not be re-added)
        self.deleted_batch_ids: set = set()
        
        # Set to track currently active batch known_ids (from last update)
        self.active_batch_ids: set = set()

        # Set to track FX safety zones that have already been added (persist across updates)
        # This prevents creating duplicate 170x170 safety obstacles for the same FX known_id.
        self.fx_safety_added: set = set()

        # Debounce repeated per-SX locked-baseline requests when source is flapping.
        self.baseline_request_cooldown_sec = float(
            params.get('rates', {}).get('baseline_request_cooldown_sec', 1.5)
        )
        self._last_locked_baseline_request_s: Dict[str, float] = {}
        
        # Set to track batches that already have obstacles created (prevents duplicates)
        self.batches_with_obstacles: set = set()
        self.manual_putdown_batch_ids: set = set()
        self.manual_pattern_overrides: Dict[str, str] = {}
        self.synthetic_override_batch_ids: set = set()

        # Retry management for auto update_map
        self.retry_timer = None
        self.retry_timer_active = False
        self.retry_interval_sec = params.get('rates', {}).get('update_map_retry_sec', 1.0)
        self.retry_pending = 0
        self.retry_client = None
        self.retry_service_name = ""

        # Automatic source handover guard (web -> aruco)
        self.source_switch_in_progress = False

        # Short-pattern retry: re-check batches whose locked pattern has < 4 elements
        self.short_pattern_ids: set = set()
        self.tentative_baseline_ids: set = set()
        self.short_pattern_retry_timer = None
        self.short_pattern_retry_active = False
        self.short_pattern_retry_interval_sec = params.get('rates', {}).get('short_pattern_retry_sec', 2.0)

        # Post-lock updates via /aruco_mapper/all_batches
        self.all_batches_sub = None
        self.all_batches_sub_active = False
        self.all_batches_topic = params.get('topics', {}).get('all_batches', '/aruco_mapper/all_batches')
        self.pending_baseline_request_ids: set = set()
        self.fx_pending_baseline_batches: Dict[str, Batch] = {}

        # Game-start driven periodic update_map flow
        self.start_game_requested = False
        self.game_update_timer_active = False
        self.game_update_pending = 0
        self.game_update_interval_sec = params.get('rates', {}).get('start_game_update_map_sec', 1.0)
        self.game_start_time_ns: Optional[int] = None
        self.game_85s_triggered = False
        self.game_timeout_stop_sec = float(params.get('rates', {}).get('start_game_timeout_sec', 105.0))
        self.game_timeout_stopped = False

        # Enemy obstacle freshness management (/obstacle_relative)
        self.enemy_obstacle_timeout_sec = params.get('rates', {}).get('enemy_obstacle_timeout_sec', 1.0)
        self.last_enemy_update_ns: Optional[int] = None
        self.enemy_from_web: bool = False

        # Team color used to mirror 85s timed PAMI walls.
        # yellow: default coordinates, blue: mirrored on x-axis (y mirrored around horizontal center axis).
        self.team_color: str = 'yellow'
        field_width = float(params.get('map', {}).get('width_mm', 3000.0))
        field_height = float(params.get('map', {}).get('height_mm', 2000.0))
        self._symmetry_axis_y_mm = 1500.0  # Horizontal center axis for y-symmetry
        
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

        node.create_subscription(
            String, '/hybrid_astar/batch_pattern_override_cmd',
            self.handle_pattern_override_command, 10)
        
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

        game_update_interval = max(self.game_update_interval_sec, 0.1)
        self.game_update_timer = self.node.create_timer(
            game_update_interval, self._game_update_timer_callback
        )

        self.node.create_timer(1.0, self._debug_batches_members_callback)
        
        self.node.get_logger().info('🗺️  Map Manager initialized')
        self.node.get_logger().info(
            f"⚙️  activate_lost_batch_deleting={ACTIVATE_LOST_BATCH_DELETING}"
        )

    def _publish_beacon_status(self, value: int) -> None:
        msg = Int32()
        msg.data = int(value)
        self.beacon_status_pub.publish(msg)
        self.node.get_logger().info(f"📡 Published beacon status from map manager: {msg.data}")

    def _all_active_batches_baseline_locked(self) -> bool:
        """Return True when all active non-manual batches have locked baselines."""
        required_locked_ids = {
            bid for bid in self.active_batch_ids
            if bid not in self.manual_putdown_batch_ids
        }
        return required_locked_ids.issubset(self.locked_baseline_all_batch_ids)

    def _try_publish_beacon_ready(self) -> None:
        """Publish beacon-ready only once, after baseline locking is fully stable."""
        if self.beacon_ready_published:
            self._maybe_start_all_batches_subscription()
            return

        if not self.baselines_locked:
            return

        if self.known_ids_to_retry or self.pending_baseline_lock_ids or self.short_pattern_ids or self.tentative_baseline_ids:
            return

        if not self._all_active_batches_baseline_locked():
            self.node.get_logger().info(
                "⏳ Beacon ready not published yet: waiting for all active batches to be baseline locked"
            )
            return

        self._publish_beacon_status(1)
        self.beacon_ready_published = True
        self._maybe_start_all_batches_subscription()

    def _maybe_start_all_batches_subscription(self) -> None:
        """Start /aruco_mapper/all_batches subscription after baseline lock is fully stable."""
        if self.all_batches_sub_active:
            return

        if not self.baselines_locked:
            return

        if self.known_ids_to_retry or self.pending_baseline_lock_ids or self.short_pattern_ids or self.tentative_baseline_ids:
            return

        if not self._all_active_batches_baseline_locked():
            return

        self.all_batches_sub = self.node.create_subscription(
            ArucoMapperState,
            self.all_batches_topic,
            self._handle_all_batches_topic,
            10
        )
        self.all_batches_sub_active = True
        self.node.get_logger().info(
            f"📡 Subscribed to {self.all_batches_topic} for post-lock batch updates"
        )

    def _ensure_all_batches_subscription(self, reason: str) -> None:
        """Subscribe to /aruco_mapper/all_batches if not already active."""
        if self.all_batches_sub_active:
            return

        self.all_batches_sub = self.node.create_subscription(
            ArucoMapperState,
            self.all_batches_topic,
            self._handle_all_batches_topic,
            10
        )
        self.all_batches_sub_active = True
        self.node.get_logger().info(
            f"📡 Subscribed to {self.all_batches_topic} ({reason})"
        )

    def _invalidate_sx_baseline_lock(self, known_id: str, state: str) -> None:
        """Clear SX baseline-lock markers when mapper state regresses.

        This allows requesting locked baseline again for the same known_id.
        """
        if not known_id.startswith('s'):
            return

        was_locked = (
            known_id in self.locked_baseline_batch_ids
            or known_id in self.locked_baseline_all_batch_ids
        )

        if was_locked:
            self.node.get_logger().info(
                f"ℹ️  SX '{known_id}' regressed to {state}, but baseline is already locked; keeping it locked until LOST or put_down"
            )
            return

        self.locked_baseline_batch_ids.discard(known_id)
        self.locked_baseline_all_batch_ids.discard(known_id)
        self.pending_baseline_lock_ids.discard(known_id)

        if was_locked:
            self.node.get_logger().warn(
                f"⚠️  SX '{known_id}' regressed to {state}; clearing baseline lock markers so it can be re-locked"
            )

    @staticmethod
    def _parse_batch_state(state_value: str) -> Tuple[str, str]:
        """
        Parse mapper batch state and extract optional status suffix.

        Examples:
          TRACKING      -> (TRACKING, '')
          TRACKING_NP   -> (TRACKING, 'NP')
          EMERGING_P    -> (EMERGING, 'P')
          TRACKING_T    -> (TRACKING, 'T')
          TRACKING_C    -> (TRACKING, 'C')
        """
        normalized = str(state_value or "").strip().upper()
        if not normalized:
            return "", ""
        if '_' in normalized:
            base, suffix = normalized.rsplit('_', 1)
            if suffix in ('NP', 'P', 'T', 'C'):
                return base, suffix
        return normalized, ""

    @staticmethod
    def _is_valid_pattern(pattern: str) -> bool:
        normalized = str(pattern or '').strip().lower()
        return len(normalized) == 4 and all(c in ('y', 'b') for c in normalized)

    def _apply_manual_pattern_override(self, batch: Batch) -> None:
        """Replace batch.pattern, direction, and member colors with manual override when available."""
        known_id = str(getattr(batch, 'known_id', '') or '').strip().lower()
        if not known_id:
            return
        override = self.manual_pattern_overrides.get(known_id)
        if not override:
            return

        batch.pattern = override
        direction = self._default_direction_for_known_id(known_id)
        batch.direction = direction

        # Update member colors by sorting members positionally then assigning from pattern
        members = list(batch.members) if batch.members else []
        if len(members) >= 4 and len(override) >= 4:
            if direction == 'horizontal':
                members_sorted = sorted(members, key=lambda m: getattr(m.map_pos, 'x', 0.0))
            else:
                members_sorted = sorted(members, key=lambda m: getattr(m.map_pos, 'y', 0.0))
            for i, member in enumerate(members_sorted):
                member.color = 'yellow' if override[i] == 'y' else 'blue'

    def _default_direction_for_known_id(self, known_id: str) -> str:
        """Return a deterministic default direction for a known batch id."""
        known_id = str(known_id or '').strip().lower()
        if known_id.startswith('s'):
            if known_id in ('s1', 's2', 's7', 's8'):
                return 'horizontal'
            return 'vertical'
        return 'vertical'

    def _create_synthetic_batch_from_override(self, known_id: str, pattern: str) -> bool:
        """Create a synthetic batch with 4 members from manual override data.

        This is used when no live batch is currently active but the operator wants
        to visualize/update a batch pattern immediately.
        """
        known_id = str(known_id or '').strip().lower()
        pattern = str(pattern or '').strip().lower()
        if not known_id or not self._is_valid_pattern(pattern):
            return False

        direction = self._default_direction_for_known_id(known_id)
        batch = Batch()
        batch.batch_id = int(time.time() * 1000) % 1000000000
        batch.known_id = known_id
        batch.direction = direction
        batch.count = 4
        batch.state = 'TRACKING_C' if known_id.startswith('s') else 'TRACKING'
        batch.pattern = pattern

        center_x_m = None
        center_y_m = None
        if known_id.startswith('s'):
            if known_id not in self.predefined_batch_positions:
                self.node.get_logger().warn(
                    f"⚠️  Cannot synthesize override for '{known_id}': unknown SX predefined position"
                )
                return False
            batch_pos = self.predefined_batch_positions[known_id]
            center_x_m = float(batch_pos['x'])
            center_y_m = float(batch_pos['y'])
        else:
            self.node.get_logger().warn(
                f"⚠️  Cannot synthesize override for '{known_id}': SX only is supported"
            )
            return False

        center = Point()
        center.x = center_x_m
        center.y = center_y_m
        center.z = 0.0
        batch.center = center

        offsets_mm = (
            [(-75.0, 0.0), (-25.0, 0.0), (25.0, 0.0), (75.0, 0.0)]
            if direction == 'horizontal'
            else [(0.0, -75.0), (0.0, -25.0), (0.0, 25.0), (0.0, 75.0)]
        )

        for idx, (dx_mm, dy_mm) in enumerate(offsets_mm):
            member = BatchMember()
            member.aruco_id = 9000 + idx
            member.color = 'yellow' if pattern[idx] == 'y' else 'blue'

            member_pos = Point()
            member_pos.x = center_x_m + (dx_mm / 1000.0)
            member_pos.y = center_y_m + (dy_mm / 1000.0)
            member_pos.z = 0.0
            member.map_pos = member_pos
            batch.members.append(member)

        with self.batches_lock:
            if known_id.startswith('s'):
                self.batches_sx[known_id] = batch
                self.batches_sx_baseline_status_type[known_id] = 'C'
                self.locked_baseline_batch_ids.add(known_id)
                self.locked_baseline_all_batch_ids.add(known_id)
                self.pending_baseline_lock_ids.discard(known_id)
                self.tentative_baseline_ids.discard(known_id)
            self.active_batch_ids.add(known_id)
            self.batches_with_obstacles.discard(known_id)
            self.deleted_batch_ids.discard(known_id)
            self.synthetic_override_batch_ids.add(known_id)

        self.node.get_logger().info(
            f"🧩 Created synthetic batch for override: {known_id} pattern={pattern} direction={direction}"
        )
        return True

    def _reset_batch_for_override_clear(self, known_id: str) -> None:
        """Drop one batch from local state so it can be rebuilt from live source.

        Unlike `delete_batch()`, this does NOT mark the batch as manually deleted.
        """
        known_id = str(known_id or '').strip().lower()
        if not known_id:
            return

        with self.obstacles_lock:
            self.obstacles = [
                obs for obs in self.obstacles
                if not (obs.id.startswith(f"fx_{known_id}_") or obs.id.startswith(f"sx_{known_id}_"))
            ]
            self.obstacle_sources = {
                k: v for k, v in self.obstacle_sources.items()
                if not (k.startswith(f"fx_{known_id}_") or k.startswith(f"sx_{known_id}_"))
            }

        with self.batches_lock:
            self.batches_fx.pop(known_id, None)
            self.batches_fx_status_type.pop(known_id, None)
            self.batches_sx.pop(known_id, None)
            self.batches_sx_status_type.pop(known_id, None)
            self.batches_sx_baseline_status_type.pop(known_id, None)
            self.batches_with_obstacles.discard(known_id)
            self.locked_baseline_batch_ids.discard(known_id)
            self.locked_baseline_all_batch_ids.discard(known_id)
            self.pending_baseline_lock_ids.discard(known_id)
            self.short_pattern_ids.discard(known_id)
            self.tentative_baseline_ids.discard(known_id)
            self.manual_putdown_batch_ids.discard(known_id)
            self.synthetic_override_batch_ids.discard(known_id)
            self.active_batch_ids.discard(known_id)
            self.known_ids_to_retry.discard(known_id)
            self.deleted_batch_ids.discard(known_id)

    def handle_pattern_override_command(self, msg: String) -> None:
        """Handle manual pattern override commands from web_handler.

        Expected JSON payload:
          {"action":"set","known_id":"s1","pattern":"byyb"}
          {"action":"clear","known_id":"s1"}
          {"action":"clear_all"}
        """
        try:
            data = json.loads(msg.data or '{}')
            action = str(data.get('action', '')).strip().lower()
            known_id = str(data.get('known_id', '')).strip().lower()
            pattern = str(data.get('pattern', '')).strip().lower()

            if action == 'set':
                if not known_id:
                    raise ValueError("Missing known_id for override set")
                if not known_id.startswith('s'):
                    raise ValueError(
                        f"Manual pattern override supports SX only (got '{known_id}')"
                    )
                if not self._is_valid_pattern(pattern):
                    raise ValueError(
                        f"Invalid pattern '{pattern}' (expected 4 chars using y/b)"
                    )
                self.manual_pattern_overrides[known_id] = pattern
                self.node.get_logger().info(
                    f"🎛️  Manual pattern override set: {known_id} -> {pattern}"
                )
                rebuilt = self._rebuild_overridden_batch_if_present(known_id)
                if not rebuilt and self._create_synthetic_batch_from_override(known_id, pattern):
                    self._rebuild_overridden_batch_if_present(known_id)
                return

            if action == 'clear':
                if not known_id:
                    raise ValueError("Missing known_id for override clear")
                if not known_id.startswith('s'):
                    raise ValueError(
                        f"Manual pattern override supports SX only (got '{known_id}')"
                    )
                removed = self.manual_pattern_overrides.pop(known_id, None)
                if removed is None:
                    self.node.get_logger().info(
                        f"ℹ️  No manual pattern override found for '{known_id}'"
                    )
                else:
                    self.node.get_logger().info(
                        f"🧽 Cleared manual pattern override for '{known_id}'"
                    )

                self._reset_batch_for_override_clear(known_id)
                success, message, _service_name = self._trigger_update_map_async(can_lock_baseline=False)
                if success:
                    self.node.get_logger().info(
                        f"🔄 Refresh triggered after clearing override for '{known_id}': {message}"
                    )
                else:
                    self.node.get_logger().warn(
                        f"⚠️ Could not refresh map after clearing override for '{known_id}': {message}"
                    )
                return

            if action == 'clear_all':
                overridden_ids = set(self.manual_pattern_overrides.keys()) | set(self.synthetic_override_batch_ids)
                count = len(self.manual_pattern_overrides)
                self.manual_pattern_overrides.clear()
                for sid in list(overridden_ids):
                    self._reset_batch_for_override_clear(sid)
                self.node.get_logger().info(
                    f"🧽 Cleared all manual pattern overrides ({count})"
                )

                success, message, _service_name = self._trigger_update_map_async(can_lock_baseline=False)
                if success:
                    self.node.get_logger().info(
                        f"🔄 Refresh triggered after clear_all overrides: {message}"
                    )
                else:
                    self.node.get_logger().warn(
                        f"⚠️ Could not refresh map after clear_all overrides: {message}"
                    )
                return

            raise ValueError(f"Unsupported override action '{action}'")

        except Exception as e:
            self.node.get_logger().error(f"❌ Invalid pattern override command: {e}")

    def _rebuild_overridden_batch_if_present(self, known_id: str) -> bool:
        """If batch is already active, apply override and rebuild its obstacles."""
        target_batch = None
        source = None
        with self.batches_lock:
            if known_id in self.batches_sx:
                target_batch = self.batches_sx[known_id]
                source = 'sx'
            elif known_id in self.batches_fx:
                target_batch = self.batches_fx[known_id]
                source = 'fx'

            if target_batch is None:
                return False

            self._apply_manual_pattern_override(target_batch)
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

        if target_batch is None or source is None:
            return False

        fake_state = ArucoMapperState()
        fake_state.stamp = self.node.get_clock().now().to_msg()
        fake_state.initialized = True
        fake_state.batches = [target_batch]
        self._handle_aruco_mapper_state(fake_state, source)
        return True
    
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
        
        # Stop post-lock all-batches subscription
        if self.all_batches_sub_active and self.all_batches_sub is not None:
            try:
                self.node.destroy_subscription(self.all_batches_sub)
            except Exception:
                pass
        self.all_batches_sub = None
        self.all_batches_sub_active = False
        self.pending_baseline_request_ids.clear()
        self.fx_pending_baseline_batches.clear()

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
            self.batches_fx_status_type.clear()
            self.batches_sx_status_type.clear()
            self.batches_sx_baseline_status_type.clear()
            self.batches_with_obstacles.clear()  # Clear tracking
            self.manual_putdown_batch_ids.clear()
            self.manual_pattern_overrides.clear()
            self.synthetic_override_batch_ids.clear()
        
        # Reset state flags
        self.first_update_map = True
        self.first_update_empty_retry_count = 0
        self.baselines_locked = False
        self.beacon_ready_published = False
        self.locked_baseline_batch_ids.clear()
        self.locked_baseline_all_batch_ids.clear()
        self.pending_baseline_lock_ids.clear()
        self.baseline_lock_pending = False
        self.start_game_requested = False
        self.game_start_time_ns = None
        self.game_85s_triggered = False
        self.game_timeout_stopped = False
        self._stop_game_update_timer()
        self.known_ids_to_retry.clear()
        self.deleted_batch_ids.clear()
        self.active_batch_ids.clear()
        self.pending_baseline_request_ids.clear()
        self.fx_pending_baseline_batches.clear()
        self.short_pattern_ids.clear()
        self.tentative_baseline_ids.clear()
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
                self.batches_fx_status_type.pop(known_id, None)
                deleted = True
                self.node.get_logger().info(f"  ✅ Removed FX batch '{known_id}'")
            if known_id in self.batches_sx:
                del self.batches_sx[known_id]
                self.batches_sx_status_type.pop(known_id, None)
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

        # Any members already tracked for this zone (beacon or prior manual putdown)
        zone_had_members = (
            existing_batch is not None
            and len(getattr(existing_batch, 'members', [])) > 0
        )

        existing_members = list(existing_batch.members) if (existing_is_manual and existing_batch is not None) else []
        existing_count = len(existing_members)
        if existing_count >= 8:
            return False, f"Batch {known_id} already contains 8 manually put-down members", [], known_id

        # Normal 4-member row at the computed batch center
        member_positions = []
        for lat in lateral_offsets:
            member_positions.append((
                batch_center_x_mm + lat * vx,
                batch_center_y_mm + lat * vy,
            ))

        # When the zone already had obstacles, add a second row of 4 yellow members
        # 100 mm further forward (same heading) to represent the pre-existing stack.
        extra_row_offset_mm = 100.0
        extra_row_positions = []
        if zone_had_members and not existing_is_manual:
            for lat in lateral_offsets:
                extra_row_positions.append((
                    batch_center_x_mm + extra_row_offset_mm * ux + lat * vx,
                    batch_center_y_mm + extra_row_offset_mm * uy + lat * vy,
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

        extra_start_idx = len(all_members)
        for idx, (member_x_mm, member_y_mm) in enumerate(extra_row_positions, start=extra_start_idx):
            member = BatchMember()
            member.aruco_id = idx
            member.color = 'yellow'
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
        extra_pattern = 'yyyy' if extra_row_positions else ''
        batch.pattern = existing_pattern + ('yyyy' if color == 'yellow' else 'bbbb') + extra_pattern

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

            # 200x200 center safety — always on first registration, at the predefined FX zone position
            if existing_count == 0:
                center_safety_id = f"fx_{known_id}_safety_center"
                fx_center_x_mm = self.putdown_fx_positions_mm[known_id]['x']
                fx_center_y_mm = self.putdown_fx_positions_mm[known_id]['y']

                center_safety_obs = Obstacle()
                center_safety_obs.id = center_safety_id
                center_safety_obs.type = 'safety_zone'
                center_safety_obs.color = 'wall'
                center_safety_obs.width_mm = 200.0
                center_safety_obs.height_mm = 200.0
                center_safety_obs.pose.x = fx_center_x_mm
                center_safety_obs.pose.y = fx_center_y_mm
                center_safety_obs.pose.theta = 0.0
                center_safety_obs.detected_at = stamp
                self.obstacles.append(center_safety_obs)
                self.obstacle_sources[center_safety_obs.id] = 'put_down_safety'
                created_ids.append(center_safety_obs.id)

                self.node.get_logger().info(
                    f"🛡️ Added center safety for {known_id}: 200mm x 200mm at ({fx_center_x_mm:.1f}, {fx_center_y_mm:.1f})mm"
                )

            # 170x170 projected safety at 375mm — only when zone had pre-existing members
            safety_obs_id = f"fx_{known_id}_safety"
            safety_already_added = any(obs.id == safety_obs_id for obs in self.obstacles)
            if zone_had_members and not safety_already_added:
                heading_dot = (batch_center_x_mm - robot_x_mm) * ux + (batch_center_y_mm - robot_y_mm) * uy
                safety_sign = 1.0 if heading_dot >= 0.0 else -1.0
                safety_projection_mm = 375.0
                safety_center_x_mm = robot_x_mm + safety_projection_mm * safety_sign * ux
                safety_center_y_mm = robot_y_mm + safety_projection_mm * safety_sign * uy

                safety_obs = Obstacle()
                safety_obs.id = safety_obs_id
                safety_obs.type = 'safety_zone'
                safety_obs.color = 'wall'
                safety_obs.width_mm = 170.0
                safety_obs.height_mm = 170.0
                safety_obs.pose.x = safety_center_x_mm
                safety_obs.pose.y = safety_center_y_mm
                safety_obs.pose.theta = robot_theta
                safety_obs.detected_at = stamp
                self.obstacles.append(safety_obs)
                self.obstacle_sources[safety_obs.id] = 'put_down_safety'
                created_ids.append(safety_obs.id)

                self.node.get_logger().info(
                    f"🛡️ Added projected safety for {known_id}: 170mm x 170mm at ({safety_center_x_mm:.1f}, {safety_center_y_mm:.1f})mm "
                    f"(375mm {'forward' if safety_sign > 0 else 'backward'} from robot)"
                )

        if extra_row_positions:
            self.node.get_logger().info(
                f"➕ Added extra yellow row for {known_id}: 4 members at +{extra_row_offset_mm:.0f}mm forward "
                f"(zone had pre-existing obstacles)"
            )

        with self.batches_lock:
            self.batches_fx[known_id] = batch
            self.batches_fx_status_type.pop(known_id, None)
            self.batches_with_obstacles.add(known_id)
            self.manual_putdown_batch_ids.add(known_id)
            self.active_batch_ids.add(known_id)
            # Cancel any pending beacon-driven retry for this known_id so the
            # short-pattern / EMERGING retry loops don't overwrite the putdown.
            self.tentative_baseline_ids.discard(known_id)
            self.short_pattern_ids.discard(known_id)
            self.known_ids_to_retry.discard(known_id)

        self.publish_obstacles(self.get_active_obstacles())
        extra_suffix = f" + {len(extra_row_positions)} extra yellow row" if extra_row_positions else ""
        if existing_count > 0:
            return True, f"Appended put_down to {known_id} ({existing_count} -> {len(all_members)} members){extra_suffix}", created_ids, known_id
        return True, f"Registered put_down batch as {known_id}{extra_suffix}", created_ids, known_id

    def _reconcile_manual_putdown_batches(self, received_batch_ids: set, lost_batch_ids: set) -> set:
        """
        Keep manual put-down FX batches even if beacon reports LOST.
        Beacon updates must not remove/overwrite manual put_down batches.

        Returns:
            Set of manual put-down known_ids that should be preserved this cycle.
        """
        with self.batches_lock:
            manual_ids = set(self.manual_putdown_batch_ids)

        # Preserve manual batches by default, including when beacon reports LOST.
        # Requirement: if batch was put down manually by us, do not update/remove it.
        retained_manual_ids = set(manual_ids)
        lost_manual_ids = manual_ids & lost_batch_ids

        if lost_manual_ids:
            self.node.get_logger().info(
                f"📌 Ignoring LOST for manual put_down batches (preserved): {lost_manual_ids}"
            )

        if retained_manual_ids:
            self.node.get_logger().info(
                f"📌 Preserving manual put_down batches from beacon refresh: {retained_manual_ids}"
            )

        return retained_manual_ids

    def _remove_lost_batches(self, batch_ids: set, reason: str = "reported LOST") -> None:
        """Hard-remove lost batches (except manually preserved ones handled elsewhere)."""
        if not batch_ids:
            return

        # Ignore no-op LOST sets to avoid flapping logs like
        # "obstacles_removed=0" for already-removed batches.
        with self.batches_lock:
            tracked_batch_ids = (
                set(self.batches_fx.keys())
                | set(self.batches_sx.keys())
                | set(self.batches_with_obstacles)
                | set(self.locked_baseline_batch_ids)
                | set(self.locked_baseline_all_batch_ids)
                | set(self.pending_baseline_lock_ids)
                | set(self.short_pattern_ids)
                | set(self.active_batch_ids)
                | set(self.known_ids_to_retry)
            )

        relevant_batch_ids = set(batch_ids) & tracked_batch_ids
        if not relevant_batch_ids:
            with self.obstacles_lock:
                for obs in self.obstacles:
                    obs_id = str(obs.id)
                    if not (obs_id.startswith('fx_') or obs_id.startswith('sx_')):
                        continue
                    parts = obs_id.split('_', 2)
                    if len(parts) >= 3 and parts[1] in batch_ids:
                        relevant_batch_ids.add(parts[1])

        if not relevant_batch_ids:
            return

        with self.obstacles_lock:
            old_count = len(self.obstacles)
            self.obstacles = [
                obs for obs in self.obstacles
                if not any(
                    obs.id.startswith(f"fx_{bid}_") or obs.id.startswith(f"sx_{bid}_")
                    for bid in relevant_batch_ids
                )
            ]
            removed_obs = old_count - len(self.obstacles)
            self.obstacle_sources = {
                k: v for k, v in self.obstacle_sources.items()
                if not any(k.startswith(f"fx_{bid}_") or k.startswith(f"sx_{bid}_") for bid in relevant_batch_ids)
            }

        with self.batches_lock:
            for bid in relevant_batch_ids:
                self.batches_fx.pop(bid, None)
                self.batches_fx_status_type.pop(bid, None)
                self.batches_sx.pop(bid, None)
                self.batches_sx_status_type.pop(bid, None)
                self.batches_sx_baseline_status_type.pop(bid, None)
                self.batches_with_obstacles.discard(bid)
                self.locked_baseline_batch_ids.discard(bid)
                self.locked_baseline_all_batch_ids.discard(bid)
                self.pending_baseline_lock_ids.discard(bid)
                self.short_pattern_ids.discard(bid)
                self.tentative_baseline_ids.discard(bid)
                self.active_batch_ids.discard(bid)
                self.known_ids_to_retry.discard(bid)
                self._last_locked_baseline_request_s.pop(bid, None)
                self.pending_baseline_request_ids.discard(bid)
                self.fx_pending_baseline_batches.pop(bid, None)

        self.node.get_logger().info(
            f"🗑️  Removed {len(relevant_batch_ids)} batch(es) {reason}: {relevant_batch_ids} (obstacles_removed={removed_obs})"
        )
    
    def handle_obstacle_relative(self, msg: PoseStamped):
        """Handle incoming obstacle from /obstacle_relative topic (only keeps one enemy)"""
        if self.game_timeout_stopped:
            return
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
        if self.game_timeout_stopped:
            return
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
        
        crates = list(obstacles)
        
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
            if obs['color'] == 'wall':
                crate_marker.pose.position.z = 0.15  # make wall easier to see
            else:
                crate_marker.pose.position.z = 0.03  # 30mm high (half of 150mm)
            crate_marker.pose.orientation = self.yaw_to_quaternion(obs.get('theta', 0.0))
            
            # Size
            crate_marker.scale.x = obs['width'] / 1000.0
            crate_marker.scale.y = obs['height'] / 1000.0
            if obs['color'] == 'wall':
                crate_marker.scale.z = 0.30
            else:
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
            elif obs['color'] == 'wall':
                crate_marker.color.r = 0.5
                crate_marker.color.g = 0.5
                crate_marker.color.b = 0.5
                crate_marker.color.a = 0.9
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
        success, message, _service_name = self._trigger_update_map_async(can_lock_baseline=True)
        response.success = success
        response.message = message
        return response

    def handle_start_game(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Service handler: arm game flow and start periodic update_map only after
        baseline lock is complete.
        """
        self.start_game_requested = True
        self._ensure_all_batches_subscription(reason="start_game")
        # Start/restart game-time counting from this request.
        now = self.node.get_clock().now()
        self.game_start_time_ns = int(now.nanoseconds)
        self.game_85s_triggered = False
        self.game_timeout_stopped = False

        if self.game_update_timer_active:
            # Reset and restart the timer
            self._stop_game_update_timer()
            self._start_game_update_timer()
            response.success = True
            response.message = (
                f"Game timer restarted: update_map every {max(self.game_update_interval_sec, 0.1):.1f}s"
            )
            return response

        # If already locked, start immediately.
        if self.baselines_locked:
            self._start_game_update_timer()
            response.success = True
            response.message = (
                f"Game started: periodic update_map enabled every {max(self.game_update_interval_sec, 0.1):.1f}s"
            )
            return response

        # Otherwise trigger initial update flow; timer starts automatically once locked.
        success, message, _service_name = self._trigger_update_map_async(can_lock_baseline=True)
        if not success:
            response.success = False
            response.message = f"start_game failed to trigger initial update_map: {message}"
            return response

        response.success = True
        response.message = (
            "start_game accepted: initial update_map triggered; timer will start after baseline lock completes"
        )
        return response

    def _start_game_update_timer(self):
        if self.game_timeout_stopped:
            self.node.get_logger().warn(
                f"⏹️ Game update timer start ignored: game already timed out at {self.game_timeout_stop_sec:.1f}s"
            )
            return
        if self.game_update_timer_active:
            return
        self.game_update_timer_active = True
        self.node.get_logger().info(
            f"🎮 Game update timer started: update_map every {max(self.game_update_interval_sec, 0.1):.1f}s"
        )

    def _stop_game_update_timer(self):
        if not self.game_update_timer_active:
            return
        self.game_update_timer_active = False
        self.game_update_pending = 0
        self.node.get_logger().info("🛑 Game update timer stopped")

    def handle_set_team_color(self, request: SetTeamColor.Request, response: SetTeamColor.Response) -> SetTeamColor.Response:
        """Service handler to set active team color for timed PAMI wall symmetry."""
        color = str(getattr(request, 'color', '') or '').strip().lower()
        if color not in ('yellow', 'blue'):
            response.success = False
            response.message = "Invalid color. Use 'yellow' or 'blue'."
            return response

        self.team_color = color
        response.success = True
        response.message = f"Team color set to {self.team_color}"
        self.node.get_logger().info(
            f"🎨 Team color updated: {self.team_color} (85s wall symmetry axis y={self._symmetry_axis_y_mm:.1f}mm)"
        )
        return response

    def _add_wall_sima_obstacle(self) -> None:
        """Add/update a timed game obstacle used to block a busy zone."""
        wall_id = 'wall_sima'
        wall_id_2 = 'wall_sima_2'
        wall_id_3 = 'wall_3'

        # Requested values:
        # - center in mm
        # - width/height provided in meters
        center_x_mm = 1500.0
        center_y_mm = 800.0
        width_mm = 600.0
        height_mm = 1500.0

        # Second requested wall
        center2_x_mm = 550.0
        center2_y_mm = 1250.0
        width2_mm = 300.0
        height2_mm = 300.0

        # Third wall: block escape toward (900, 400)
        center3_x_mm = 900.0
        center3_y_mm = 400.0
        width3_mm = 300.0
        height3_mm = 300.0

        # For blue, apply x-axis symmetry: mirror y around horizontal center axis.
        if self.team_color == 'blue':
            center_y_mm = (2.0 * self._symmetry_axis_y_mm) - center_y_mm
            center2_y_mm = (2.0 * self._symmetry_axis_y_mm) - center2_y_mm
            center3_y_mm = (2.0 * self._symmetry_axis_y_mm) - center3_y_mm

        with self.obstacles_lock:
            # Remove any previous instance with same id so we can re-add cleanly.
            self.obstacles = [obs for obs in self.obstacles if obs.id not in (wall_id, wall_id_2, wall_id_3)]

            obs = Obstacle()
            obs.id = wall_id
            obs.type = 'wall'
            obs.color = 'wall'
            obs.pose.x = center_x_mm
            obs.pose.y = center_y_mm
            obs.pose.theta = 0.0
            obs.width_mm = width_mm
            obs.height_mm = height_mm
            obs.detected_at = self.node.get_clock().now().to_msg()

            self.obstacles.append(obs)
            self.obstacle_sources[wall_id] = 'game_zone'

            obs2 = Obstacle()
            obs2.id = wall_id_2
            obs2.type = 'wall'
            obs2.color = 'wall'
            obs2.pose.x = center2_x_mm
            obs2.pose.y = center2_y_mm
            obs2.pose.theta = 0.0
            obs2.width_mm = width2_mm
            obs2.height_mm = height2_mm
            obs2.detected_at = self.node.get_clock().now().to_msg()

            self.obstacles.append(obs2)
            self.obstacle_sources[wall_id_2] = 'game_zone'

            obs3 = Obstacle()
            obs3.id = wall_id_3
            obs3.type = 'wall'
            obs3.color = 'wall'
            obs3.pose.x = center3_x_mm
            obs3.pose.y = center3_y_mm
            obs3.pose.theta = 0.0
            obs3.width_mm = width3_mm
            obs3.height_mm = height3_mm
            obs3.detected_at = self.node.get_clock().now().to_msg()

            self.obstacles.append(obs3)
            self.obstacle_sources[wall_id_3] = 'game_zone'

        self.node.get_logger().info(
            f"🧱 Added timed obstacle 'wall_sima' at 85s for {self.team_color} "
            f"(center=({center_x_mm:.1f}, {center_y_mm:.1f})mm, size={width_mm:.1f}x{height_mm:.1f}mm)"
        )
        self.node.get_logger().info(
            f"🧱 Added timed obstacle 'wall_sima_2' at 85s for {self.team_color} "
            f"(center=({center2_x_mm:.1f}, {center2_y_mm:.1f})mm, size={width2_mm:.1f}x{height2_mm:.1f}mm)"
        )
        self.node.get_logger().info(
            f"🧱 Added timed obstacle 'wall_3' at 85s for {self.team_color} "
            f"(center=({center3_x_mm:.1f}, {center3_y_mm:.1f})mm, size={width3_mm:.1f}x{height3_mm:.1f}mm)"
        )

    def _check_game_time_events(self) -> None:
        """Check elapsed game time and fire one-shot hooks (e.g., at 85s)."""
        if not self.start_game_requested:
            return
        if self.game_start_time_ns is None:
            return

        elapsed_s = (int(self.node.get_clock().now().nanoseconds) - int(self.game_start_time_ns)) / 1e9

        # One-shot event at 85s (hook placeholder for future actions).
        if (not self.game_85s_triggered) and elapsed_s >= 85.0:
            self.game_85s_triggered = True
            self._add_wall_sima_obstacle()
            self.node.get_logger().info("⏱️ Game reached 85s (wall_sima added)")

        # Hard stop for periodic game updates.
        if (not self.game_timeout_stopped) and elapsed_s >= self.game_timeout_stop_sec:
            self.game_timeout_stopped = True
            self.start_game_requested = False
            self._stop_game_update_timer()
            self.node.get_logger().info(
                f"⏱️ Game reached {elapsed_s:.1f}s (>= {self.game_timeout_stop_sec:.1f}s) - stopped periodic update_map"
            )

    def _game_update_timer_callback(self):
        """Periodic callback that performs the same async flow as /update_map."""
        self._check_game_time_events()

        if not self.game_update_timer_active:
            return
        if not self.baselines_locked:
            return
        if self.game_update_pending > 0:
            return

        success, message, service_name = self._trigger_update_map_async(can_lock_baseline=False)
        if not success:
            self.node.get_logger().warn(f"⚠️ Game timer update_map skipped: {message}")
            return

        self.game_update_pending += 1
        self.node.get_logger().debug(
            f"🔄 Game timer update_map triggered from {service_name}: {message}"
        )

    def _debug_batches_members_callback(self):
        if not DEBUG_BATCHES_MEMBERS:
            return
        if self.game_timeout_stopped:
            return

        with self.batches_lock:
            all_ids = set(self.batches_fx.keys()) | set(self.batches_sx.keys())

        if not all_ids:
            return

        with self.obstacles_lock:
            obstacle_ids = [obs.id for obs in self.obstacles]

        lines = ["[DEBUG_BATCHES_MEMBERS]"]
        for known_id in sorted(all_ids):
            obs_count = sum(
                1 for oid in obstacle_ids
                if f"fx_{known_id}_member" in oid or f"sx_{known_id}_member" in oid
            )
            with self.batches_lock:
                if known_id in self.batches_fx:
                    quality = self.batches_fx_status_type.get(known_id, '-')
                    batch_type = 'FX'
                else:
                    quality = self.batches_sx_status_type.get(known_id, '-')
                    batch_type = 'SX'
            has_obstacles = known_id in self.batches_with_obstacles
            lines.append(
                f"  {batch_type} '{known_id}': {obs_count} obstacle(s), "
                f"status={quality}, tracked={has_obstacles}"
            )

        self.node.get_logger().info('\n'.join(lines))

    def _trigger_update_map_async(self, can_lock_baseline: bool = True) -> tuple[bool, str, str]:
        """Trigger the same asynchronous GetStableBatches flow as /update_map."""
        # If we are currently using simulation batches and ArUco service becomes available,
        # migrate source by running: clear_map -> reset(rrr) -> update_map.
        if self.retry_service_name == "/aruco_mapper/get_stable_batches_web" and self.get_stable_batches_client.service_is_ready():
            if self.source_switch_in_progress:
                return True, "Source switch web->aruco already in progress", "/aruco_mapper/get_stable_batches"

            if self._switch_from_web_to_aruco_async():
                return True, "Switching source web->aruco (clear_map -> reset(rrr) -> update_map)", "/aruco_mapper/get_stable_batches"

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
            return False, "No batch service available", service_name

        # Remember service for auto-retry timer
        self.retry_client = client_to_use
        self.retry_service_name = service_name

        if self.first_update_map:
            if not self.first_update_start_logged:
                self.node.get_logger().info("🔄 First update_map call - requesting all batches (unlocked)")
                self.first_update_start_logged = True
            if can_lock_baseline:
                self.baseline_lock_pending = True

        stable_request = GetStableBatches.Request()
        stable_request.batch_types_filter = "sfg"
        stable_request.specific_known_id = ""
        stable_request.only_locked_baselines = False

        future = client_to_use.call_async(stable_request)
        future.add_done_callback(lambda f: self._handle_get_stable_batches_response(
            f, None, client_to_use, service_name, None, False, can_lock_baseline))

        message_type = "First" if self.first_update_map else "Incremental"
        return True, f"{message_type} map update triggered from {service_name} (processing asynchronously)", service_name
        return response

    def _switch_from_web_to_aruco_async(self) -> bool:
        """Switch active batch source from web simulation to ArUco when ArUco becomes available.

        Flow: clear_map -> reset_map with 'rrr' filter (on ArUco) -> normal update_map flow.
        """
        if self.source_switch_in_progress:
            return False

        if not self.get_stable_batches_client.service_is_ready():
            return False

        self.source_switch_in_progress = True
        self.node.get_logger().info(
            "🔁 ArUco service is now available while using WEB source - switching source: clear_map -> reset(rrr) -> update_map"
        )

        # 1) Clear current map/state first
        clear_req = Trigger.Request()
        clear_resp = Trigger.Response()
        clear_resp = self.handle_clear_map(clear_req, clear_resp)
        if not clear_resp.success:
            self.node.get_logger().error("❌ Source switch aborted: clear_map failed")
            self.source_switch_in_progress = False
            return False

        # 2) Reset with rrr filter on ArUco service
        self.retry_client = self.get_stable_batches_client
        self.retry_service_name = "/aruco_mapper/get_stable_batches"

        stable_request = GetStableBatches.Request()
        stable_request.batch_types_filter = "rrr"
        stable_request.specific_known_id = ""
        stable_request.only_locked_baselines = False

        future = self.retry_client.call_async(stable_request)
        future.add_done_callback(self._handle_source_switch_reset_done)
        return True

    def _handle_source_switch_reset_done(self, future):
        """Finalize web->aruco source switch after reset(rrr), then trigger normal update_map."""
        try:
            # Process reset response through normal map-update processing pipeline
            self._handle_get_stable_batches_response(
                future,
                None,
                self.retry_client,
                self.retry_service_name,
                None,
                False,
                False,
            )

            # 3) Continue with the usual update_map flow
            success, message, _service_name = self._trigger_update_map_async(can_lock_baseline=True)
            if success:
                self.node.get_logger().info(f"✅ Source switch completed: {message}")
            else:
                self.node.get_logger().warn(f"⚠️ Source switch follow-up update_map failed: {message}")
        except Exception as e:
            self.node.get_logger().error(f"❌ Source switch reset callback failed: {e}")
        finally:
            self.source_switch_in_progress = False

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

        # Stop post-lock all-batches subscription
        if self.all_batches_sub_active and self.all_batches_sub is not None:
            try:
                self.node.destroy_subscription(self.all_batches_sub)
            except Exception:
                pass
        self.all_batches_sub = None
        self.all_batches_sub_active = False
        self.pending_baseline_request_ids.clear()
        self.fx_pending_baseline_batches.clear()

        # Rearm initial update flow so the next /update_map call behaves like a fresh first update.
        # reset_map is a reset/reinitialization step, not a normal incremental update.
        self.first_update_map = True
        self.first_update_empty_retry_count = 0
        self.baselines_locked = False
        self.baseline_lock_pending = False
        self.beacon_ready_published = False
        self.locked_baseline_batch_ids.clear()
        self.locked_baseline_all_batch_ids.clear()
        self.pending_baseline_lock_ids.clear()
        self.start_game_requested = False
        self.game_start_time_ns = None
        self.game_85s_triggered = False
        self.game_timeout_stopped = False
        self._stop_game_update_timer()
        self.known_ids_to_retry.clear()
        self.pending_baseline_request_ids.clear()
        self.fx_pending_baseline_batches.clear()
        self.short_pattern_ids.clear()
        self.tentative_baseline_ids.clear()
        self.batches_sx_baseline_status_type.clear()
        self.manual_pattern_overrides.clear()
        self.synthetic_override_batch_ids.clear()
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
                if self._parse_batch_state(batch.state)[0] == 'LOST'
            }

            # Remove LOST batches immediately, except manual put_down batches
            # which are preserved and must not be updated/removed from beacon input.
            with self.batches_lock:
                manual_putdown_ids = set(self.manual_putdown_batch_ids)
            lost_non_manual_ids = set(lost_batch_ids) - manual_putdown_ids
            lost_preserved_ids = set()
            if lost_non_manual_ids and ACTIVATE_LOST_BATCH_DELETING:
                self._remove_lost_batches(lost_non_manual_ids, reason="reported LOST by beacon")
            elif lost_non_manual_ids:
                lost_preserved_ids = set(lost_non_manual_ids)
                self.node.get_logger().info(
                    f"📌 LOST batch deletion disabled; preserving LOST batches: {lost_preserved_ids}"
                )
            
            # Initialize variables that may be used in final summary
            disappeared_ids = set()
            
            # Remember if this is the first update (before we set it to False)
            is_first_update = can_lock_baseline and self.first_update_map and specific_known_id is None
            is_full_update = specific_known_id is None  # Full update (not a specific retry)

            # If first update returns zero batches, retry as first update (do not switch to incremental flow yet).
            if is_first_update and num_batches == 0:
                self.first_update_empty_retry_count += 1
                attempt = self.first_update_empty_retry_count
                self.baseline_lock_pending = True
                if self.first_update_empty_retry_max is None:
                    retry_suffix = "unlimited"
                else:
                    retry_suffix = str(self.first_update_empty_retry_max)
                retry_msg = (
                    f"⚠️ First update_map returned 0 batches, retrying as first update "
                    f"({attempt}/{retry_suffix})"
                )
                if attempt == 1:
                    self.node.get_logger().warn(retry_msg)
                else:
                    self.node.get_logger().debug(retry_msg)

                retry_success, retry_message, _retry_service = self._trigger_update_map_async(
                    can_lock_baseline=True
                )
                if not retry_success:
                    self.node.get_logger().error(
                        f"❌ Cannot retry first update_map: {retry_message}"
                    )
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
                    state, _ = self._parse_batch_state(batch.state)
                    if state == 'LOST':
                        # LOST batches are removed immediately and must not be processed further.
                        self.known_ids_to_retry.discard(batch.known_id)
                        skipped_count += 1
                        continue
                    if state != 'EMERGING':
                        # Accept all non-EMERGING batches (TRACKING, DEGRADED, etc.)
                        filtered_batches.append(batch)
                        non_emerging_count += 1
                    else:
                        # EMERGING: SX -> retry, FX -> retry, GX -> skip
                        if batch.known_id.startswith('s'):
                            self._invalidate_sx_baseline_lock(batch.known_id, state)
                            emerging_ids.add(batch.known_id)
                            self.node.get_logger().info(f"  ⏳ [SX] '{batch.known_id}': {state} → will retry")
                        elif batch.known_id.startswith('f') or not (batch.known_id.startswith('s') or batch.known_id.startswith('g')):
                            with self.batches_lock:
                                is_putdown = batch.known_id in self.manual_putdown_batch_ids
                            if is_putdown:
                                skipped_count += 1
                            else:
                                emerging_ids.add(batch.known_id)
                                self.node.get_logger().info(f"  ⏳ [FX] '{batch.known_id}': {state} → will retry")
                        else:
                            skipped_count += 1

                # Store EMERGING IDs for retry (SX and FX)
                self.known_ids_to_retry = emerging_ids

                self.node.get_logger().info(
                    f"📊 First update summary: {non_emerging_count} non-EMERGING, {len(emerging_ids)} EMERGING queued for retry, "
                    f"{skipped_count} GX skipped"
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
                    state, _ = self._parse_batch_state(batch.state)
                    is_fx = batch.known_id.startswith('f') or not (batch.known_id.startswith('s') or batch.known_id.startswith('g'))
                    is_sx = batch.known_id.startswith('s')
                    batch_type = 'FX' if is_fx else ('SX' if batch.known_id.startswith('s') else 'GX')

                    if state == 'LOST':
                        # LOST batches are removed immediately and must not be processed further.
                        if batch.known_id in self.known_ids_to_retry:
                            self.known_ids_to_retry.discard(batch.known_id)
                            self.node.get_logger().info(f"  🗑️  [{batch_type}] '{batch.known_id}': LOST → removed from retry")
                        skipped_count += 1
                        continue

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
                            if batch.known_id in self.known_ids_to_retry:
                                self.known_ids_to_retry.discard(batch.known_id)
                                self.node.get_logger().info(f"  🗑️  [{batch_type}] '{batch.known_id}': LOST → removed from retry")
                            skipped_count += 1
                        elif is_sx:
                            # EMERGING SX: Add to retry queue (log only on first add)
                            if (
                                batch.known_id in self.locked_baseline_batch_ids
                                or batch.known_id in self.locked_baseline_all_batch_ids
                            ):
                                if batch.known_id in self.known_ids_to_retry:
                                    self.known_ids_to_retry.discard(batch.known_id)
                                self.node.get_logger().info(
                                    f"  🔒 [{batch_type}] '{batch.known_id}': EMERGING ignored because baseline is already locked"
                                )
                                skipped_count += 1
                                continue

                            self._invalidate_sx_baseline_lock(batch.known_id, state)
                            if batch.known_id not in emerging_ids and batch.known_id not in self.known_ids_to_retry:
                                self.node.get_logger().info(f"  ⏭️  [{batch_type}] '{batch.known_id}': EMERGING → added to retry")
                            emerging_ids.add(batch.known_id)
                            skipped_count += 1
                        else:
                            # EMERGING FX -> retry (same as SX), GX -> skip
                            if is_fx:
                                with self.batches_lock:
                                    is_putdown = batch.known_id in self.manual_putdown_batch_ids
                                if not is_putdown:
                                    if batch.known_id not in emerging_ids and batch.known_id not in self.known_ids_to_retry:
                                        self.node.get_logger().info(f"  ⏭️  [{batch_type}] '{batch.known_id}': EMERGING → added to retry")
                                    emerging_ids.add(batch.known_id)
                                skipped_count += 1
                            else:
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
                    state, _ = self._parse_batch_state(batch.state)
                    is_fx = batch.known_id.startswith('f') or not (batch.known_id.startswith('s') or batch.known_id.startswith('g'))
                    batch_type = 'FX' if is_fx else ('SX' if batch.known_id.startswith('s') else 'GX')

                    if state == 'LOST':
                        if batch.known_id in self.manual_pattern_overrides:
                            continue
                        # LOST batches are removed immediately and must not be processed further.
                        self.known_ids_to_retry.discard(batch.known_id)
                        self.node.get_logger().info(
                            f"  🗑️  [{batch_type}] '{batch.known_id}': {state} ({len(batch.members)} members) → LOST, removing from retry"
                        )
                        continue
                    
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
                        if (
                            batch.known_id in self.locked_baseline_batch_ids
                            or batch.known_id in self.locked_baseline_all_batch_ids
                        ):
                            self.known_ids_to_retry.discard(batch.known_id)
                            self.node.get_logger().info(
                                f"  🔒 [{batch_type}] '{batch.known_id}': {state} ({len(batch.members)} members) → ignored because baseline is already locked"
                            )
                            continue

                        self._invalidate_sx_baseline_lock(batch.known_id, state)
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
            # If LOST deletion is disabled, treat LOST batches as still received so
            # incremental disappeared-batch cleanup does not remove them.
            received_batch_ids.update(lost_preserved_ids)
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
                            self.batches_fx_status_type.clear()
                            self.batches_sx.clear()
                            self.batches_sx_status_type.clear()
                            self.batches_sx_baseline_status_type.clear()
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
                    
                    # 2. Remove obstacles ONLY for batches that disappeared
                    # For FX batches: only remove if NOT in received list (i.e., LOST)
                    # FX batches with 4 members stay and are updated in place (not deleted every cycle)
                    fx_batches_ids_full = {batch.known_id for batch in filtered_batches 
                                          if batch.known_id.startswith('f') and len(getattr(batch, 'members', [])) == 4}
                    fx_batches_ids_full -= retained_manual_putdown_ids
                    
                    # Only remove FX batches that are incomplete (not 4 members) AND are being refreshed.
                    # Also include FX batches that report 4 members but currently have fewer than 4
                    # obstacles in the map (grew from 3→4 members between two update_map calls).
                    with self.obstacles_lock:
                        _obs_ids = [obs.id for obs in self.obstacles]
                    fx_batches_ids_incomplete = set()
                    for batch in filtered_batches:
                        if not batch.known_id.startswith('f'):
                            continue
                        member_count = len(getattr(batch, 'members', []))
                        if member_count != 4:
                            fx_batches_ids_incomplete.add(batch.known_id)
                        elif batch.known_id in self.active_batch_ids:
                            current_obs = sum(1 for oid in _obs_ids if f"fx_{batch.known_id}_member" in oid)
                            if current_obs < 4:
                                fx_batches_ids_incomplete.add(batch.known_id)
                    fx_batches_ids_incomplete -= retained_manual_putdown_ids
                    
                    batches_to_remove = disappeared_ids | fx_batches_ids_incomplete  # Disappeared + incomplete FX
                    
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
                                    self.batches_fx_status_type.pop(bid, None)
                                    self.batches_sx.pop(bid, None)
                                    self.batches_sx_status_type.pop(bid, None)
                                    self.batches_sx_baseline_status_type.pop(bid, None)
                                    self.batches_with_obstacles.discard(bid)  # Remove from tracking
                                    self.locked_baseline_batch_ids.discard(bid)
                                    self.locked_baseline_all_batch_ids.discard(bid)
                                    self.pending_baseline_lock_ids.discard(bid)
                                    self.short_pattern_ids.discard(bid)
                                    self.tentative_baseline_ids.discard(bid)
                                for bid in fx_batches_ids_incomplete:
                                    # Remove INCOMPLETE FX batches for refresh (they'll be re-added)
                                    self.batches_fx.pop(bid, None)
                                    self.batches_fx_status_type.pop(bid, None)
                                    self.batches_with_obstacles.discard(bid)  # Remove from tracking
                            
                            # Log removal
                            if disappeared_ids and fx_batches_ids_incomplete:
                                self.node.get_logger().info(
                                    f"🗑️  Removed {removed} obstacles: {len(disappeared_ids)} disappeared, {len(fx_batches_ids_incomplete)} incomplete FX to refresh"
                                )
                            elif disappeared_ids:
                                self.node.get_logger().info(
                                    f"🗑️  Removed {removed} obstacles from {len(disappeared_ids)} disappeared batches: {disappeared_ids}"
                                )
                            elif fx_batches_ids_incomplete:
                                self.node.get_logger().debug(
                                    f"🔄 Refreshing {len(fx_batches_ids_incomplete)} incomplete FX batches (removed {removed} old obstacles)"
                                )
                            if fx_batches_ids_full:
                                self.node.get_logger().debug(
                                    f"✅ Keeping {len(fx_batches_ids_full)} complete FX batches (4 members) without deletion: {fx_batches_ids_full}"
                                )
                    else:
                        # No batches to remove, initialize empty set for consistency
                        fx_batches_ids_incomplete = set()
                    
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
                    
                    # 4. Filter batches based on type and member count:
                    # - SX/GX: keep existing, only add new ones
                    # - FX with 4 members: keep existing obstacles, don't refresh (only update if lost)
                    # - FX with <4 members: always refresh (refresh every update)
                    new_batches = []
                    fx_full_kept = []
                    
                    for b in filtered_batches:
                        # FX batch with 4 members - keep if already tracked WITH 4 obstacles, don't delete/recreate
                        if b.known_id.startswith('f') and len(getattr(b, 'members', [])) == 4:
                            if b.known_id in self.active_batch_ids:
                                with self.obstacles_lock:
                                    current_obs_count = sum(
                                        1 for obs in self.obstacles
                                        if f"fx_{b.known_id}_member" in obs.id
                                    )
                                if current_obs_count >= 4:
                                    fx_full_kept.append(b.known_id)
                                    # Skip adding to new_batches - keep existing obstacles
                                    continue
                                # Obstacle count is stale (e.g. grew from 3→4 members between updates):
                                # fall through so the batch is rebuilt with 4 obstacles.
                        # SX/GX already tracked - keep existing
                        if b.known_id not in self.active_batch_ids or b.known_id.startswith('f'):
                            new_batches.append(b)
                    
                    if fx_full_kept:
                        self.node.get_logger().debug(
                            f"✅ Keeping {len(fx_full_kept)} complete FX batches without recreation: {fx_full_kept}"
                        )
                    if len(new_batches) < len(filtered_batches):
                        kept_count = len(filtered_batches) - len(new_batches)
                        self.node.get_logger().debug(
                            f"♻️  Keeping {kept_count} existing batches, processing {len(new_batches)} new/incomplete FX ones"
                        )
                    filtered_batches = new_batches
            else:
                # On retry: only remove obstacles if we have better data (TRACKING or new members)
                # For DEGRADED batches with no members, keep existing obstacles to avoid flickering
                should_update = False
                if filtered_batches:
                    for batch in filtered_batches:
                        if batch.known_id == specific_known_id:
                            parsed_state, _ = self._parse_batch_state(batch.state)
                            if parsed_state == 'TRACKING' or (batch.members and len(batch.members) > 0):
                                new_member_count = len(batch.members) if batch.members else 0

                                # For FX batches: don't downgrade obstacle count if the locked
                                # baseline service is lagging behind what all_batches already reported.
                                if specific_known_id and specific_known_id.startswith('f'):
                                    with self.batches_lock:
                                        cur = self.batches_fx.get(specific_known_id)
                                        cur_count = len(cur.members) if (cur and cur.members) else 0
                                    if new_member_count < cur_count:
                                        self.node.get_logger().debug(
                                            f"🔒 FX '{specific_known_id}': locked baseline has {new_member_count} members "
                                            f"but {cur_count} already tracked — skipping rebuild to avoid regression"
                                        )
                                        break

                                with self.batches_lock:
                                    if specific_known_id in self.manual_putdown_batch_ids:
                                        break

                                should_update = True
                                with self.obstacles_lock:
                                    old_count = len(self.obstacles)
                                    self.obstacles = [obs for obs in self.obstacles
                                                     if not (obs.id.startswith(f"fx_{specific_known_id}_") or obs.id.startswith(f"sx_{specific_known_id}_"))]
                                    removed = old_count - len(self.obstacles)
                                    # Also clear the safety-zone sentinel so it gets recreated
                                    self.fx_safety_added.discard(f"fx_{specific_known_id}_safety")

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
            elif self.game_update_pending > 0:
                self.game_update_pending = max(0, self.game_update_pending - 1)

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

    def _request_locked_baseline_for_sx(self, known_id: str) -> bool:
        """Request locked baseline for a single SX batch when it appears after initial lock.

        Returns True when a request is already pending or newly sent.
        Returns False when skipped (cooldown/service unavailable/not SX/already locked).
        """
        if not known_id.startswith('s'):
            return False
        if known_id in self.locked_baseline_batch_ids or known_id in self.pending_baseline_lock_ids:
            return True

        now_s = time.monotonic()
        last_req_s = float(self._last_locked_baseline_request_s.get(known_id, 0.0))
        if now_s - last_req_s < max(self.baseline_request_cooldown_sec, 0.0):
            return False

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
            return False

        self.pending_baseline_lock_ids.add(known_id)
        self._last_locked_baseline_request_s[known_id] = now_s
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
        return True

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
                self.node.get_logger().debug(
                    f"⏳ Locked baseline not available yet for '{known_id}'"
                )
                return

            self._apply_manual_pattern_override(target_batch)

            pattern = target_batch.pattern if target_batch.pattern else ""
            _parsed_state, baseline_status = self._parse_batch_state(target_batch.state)

            if _parsed_state == 'LOST':
                self.node.get_logger().info(
                    f"🗑️  Locked baseline for SX '{known_id}' returned LOST; stopping tentative retry"
                )
                self.short_pattern_ids.discard(known_id)
                self.tentative_baseline_ids.discard(known_id)
                self.pending_baseline_lock_ids.discard(known_id)
                if not self.short_pattern_ids and not self.tentative_baseline_ids:
                    self._stop_short_pattern_retry_timer()
                return

            if baseline_status == 'T':
                already_locked = (
                    known_id in self.locked_baseline_batch_ids
                    or known_id in self.locked_baseline_all_batch_ids
                )
                if already_locked:
                    self.short_pattern_ids.discard(known_id)
                    self.tentative_baseline_ids.discard(known_id)
                    self.node.get_logger().info(
                        f"[STICKY_BASELINE] ℹ️  Locked SX '{known_id}' came back as tentative (T); keeping existing lock until LOST or put_down"
                    )
                    if not self.short_pattern_ids and not self.tentative_baseline_ids:
                        self._stop_short_pattern_retry_timer()
                    return

                self.locked_baseline_batch_ids.discard(known_id)
                self.locked_baseline_all_batch_ids.discard(known_id)
                self.tentative_baseline_ids.add(known_id)
                with self.batches_lock:
                    self.batches_sx[known_id] = target_batch
                    self.batches_sx_baseline_status_type[known_id] = 'T'
                    _, sx_status_type = self._parse_batch_state(target_batch.state)
                    if sx_status_type in ('P', 'NP'):
                        self.batches_sx_status_type[known_id] = sx_status_type
                    else:
                        self.batches_sx_status_type.pop(known_id, None)
                    self.batches_with_obstacles.discard(known_id)

                prefix_sx = f"sx_{known_id}_"
                with self.obstacles_lock:
                    self.obstacles = [obs for obs in self.obstacles if not obs.id.startswith(prefix_sx)]
                    self.obstacle_sources = {
                        k: v for k, v in self.obstacle_sources.items() if not k.startswith(prefix_sx)
                    }

                self.node.get_logger().info(
                    f"[TEMP_BASELINE] ⏳ Locked baseline for SX '{known_id}' is tentative (status=T), retrying until status=C"
                )

                # Render tentative SX obstacle layout immediately using available data,
                # then replace it once a confirmed baseline arrives.
                sx_state = ArucoMapperState()
                sx_state.stamp = response.stable_state.stamp
                sx_state.initialized = response.stable_state.initialized
                sx_state.batches = [target_batch]
                self._handle_aruco_mapper_state(sx_state, 'sx')

                self._start_short_pattern_retry_timer()
                return

            self.locked_baseline_batch_ids.add(known_id)
            self.locked_baseline_all_batch_ids.add(known_id)
            self.tentative_baseline_ids.discard(known_id)
            with self.batches_lock:
                self.batches_sx[known_id] = target_batch
                if baseline_status in ('T', 'C'):
                    self.batches_sx_baseline_status_type[known_id] = baseline_status
                else:
                    self.batches_sx_baseline_status_type.pop(known_id, None)
                _, sx_status_type = self._parse_batch_state(target_batch.state)
                if sx_status_type in ('P', 'NP'):
                    self.batches_sx_status_type[known_id] = sx_status_type
                else:
                    self.batches_sx_status_type.pop(known_id, None)
                self.batches_with_obstacles.discard(known_id)

            self.node.get_logger().info(
                f"[FINAL_BASELINE] ✅ Locked baseline received for SX '{known_id}' pattern='{pattern}'"
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
            self._try_publish_beacon_ready()
    
    def _handle_lock_baseline_response(self, future):
        """Handle response from locked baseline patterns request."""
        try:
            response = future.result()
            if response and response.stable_state:
                num_batches = len(response.stable_state.batches)
                self.baselines_locked = True
                self.baseline_lock_pending = False
                self.node.get_logger().info(f"✅ Baseline patterns locked! Received {num_batches} locked batches")

                if self.start_game_requested and not self.game_update_timer_active:
                    self._start_game_update_timer()
                
                # Track ids to preserve as active and ids that are fully locked.
                active_batch_ids_from_lock = set()
                locked_batch_ids = set()
                sx_batches_ready_for_obstacles = []
                tentative_sx_batches_for_obstacles = []
                fx_batches_ready_for_obstacles = []
                tentative_fx_batches_for_obstacles = []
                
                # Update batches with locked patterns
                with self.batches_lock:
                    for batch in response.stable_state.batches:
                        self._apply_manual_pattern_override(batch)
                        active_batch_ids_from_lock.add(batch.known_id)
                        if batch.known_id.startswith('s'):
                            _parsed_state, baseline_status = self._parse_batch_state(batch.state)
                            self.batches_sx[batch.known_id] = batch
                            if baseline_status in ('P', 'NP'):
                                self.batches_sx_status_type[batch.known_id] = baseline_status
                            else:
                                self.batches_sx_status_type.pop(batch.known_id, None)
                            if baseline_status == 'T':
                                self.locked_baseline_batch_ids.discard(batch.known_id)
                                self.locked_baseline_all_batch_ids.discard(batch.known_id)
                                self.tentative_baseline_ids.add(batch.known_id)
                                self.batches_sx_baseline_status_type[batch.known_id] = 'T'
                                self.node.get_logger().info(
                                    f"[TEMP_BASELINE] ⏳ Tentative SX baseline: {batch.known_id} status=T, waiting for C"
                                )
                                self.batches_with_obstacles.discard(batch.known_id)
                                tentative_sx_batches_for_obstacles.append(batch)
                                continue

                            locked_batch_ids.add(batch.known_id)
                            self.locked_baseline_batch_ids.add(batch.known_id)
                            self.tentative_baseline_ids.discard(batch.known_id)
                            if baseline_status in ('T', 'C'):
                                self.batches_sx_baseline_status_type[batch.known_id] = baseline_status
                            else:
                                self.batches_sx_baseline_status_type.pop(batch.known_id, None)
                            self.node.get_logger().info(
                                f"[FINAL_BASELINE] 🔒 Locked SX pattern: {batch.known_id} = '{batch.pattern}'"
                                f"{(' (status=' + baseline_status + ')') if baseline_status else ''}"
                            )
                            if len(batch.pattern or "") >= 4:
                                sx_batches_ready_for_obstacles.append(batch)
                        elif batch.known_id.startswith('f'):
                            if batch.known_id in self.manual_putdown_batch_ids:
                                self.node.get_logger().info(
                                    f"📌 Keeping manual put_down pose for '{batch.known_id}' (skip baseline FX overwrite)"
                                )
                                continue
                            self.batches_fx[batch.known_id] = batch
                            _parsed_state, status_type = self._parse_batch_state(batch.state)
                            if status_type:
                                self.batches_fx_status_type[batch.known_id] = status_type
                            else:
                                self.batches_fx_status_type.pop(batch.known_id, None)
                            self.batches_with_obstacles.discard(batch.known_id)
                            if _parsed_state == 'BASELINE_TEMPORARY':
                                self.tentative_baseline_ids.add(batch.known_id)
                                self.locked_baseline_batch_ids.discard(batch.known_id)
                                self.locked_baseline_all_batch_ids.discard(batch.known_id)
                                if batch.members:
                                    tentative_fx_batches_for_obstacles.append(batch)
                                self.node.get_logger().info(
                                    f"[TEMP_BASELINE] ⏳ FX '{batch.known_id}': BASELINE_TEMPORARY, pattern='{batch.pattern}' → will retry"
                                )
                            else:
                                locked_batch_ids.add(batch.known_id)
                                self.tentative_baseline_ids.discard(batch.known_id)
                                if batch.members:
                                    fx_batches_ready_for_obstacles.append(batch)
                                self.node.get_logger().info(f"   🔒 Locked FX pattern: {batch.known_id} = '{batch.pattern}'")
                
                # CRITICAL: Add locked batches to active_batch_ids to prevent them from being removed
                self.active_batch_ids.update(active_batch_ids_from_lock)
                self.locked_baseline_all_batch_ids.update(locked_batch_ids)
                self.node.get_logger().info(f"   📌 Added {len(locked_batch_ids)} locked batches to active tracking")

                if self.tentative_baseline_ids:
                    self.node.get_logger().info(
                        f"⏳ Locked baseline still tentative for {len(self.tentative_baseline_ids)} batches: "
                        f"{self.tentative_baseline_ids}"
                    )
                    self._start_short_pattern_retry_timer()

                # Detect SX batches with short patterns (< 4 elements) and schedule re-checks
                for batch in response.stable_state.batches:
                    pattern = batch.pattern if batch.pattern else ""
                    if batch.known_id.startswith('s') and len(pattern) < 4:
                        self.node.get_logger().warn(
                            f"⚠️  Short pattern after baseline lock for '{batch.known_id}': "
                            f"'{pattern}' ({len(pattern)} elements, expected 4) — keeping first lock, no re-check"
                        )

                self._try_publish_beacon_ready()

                if tentative_sx_batches_for_obstacles:
                    with self.obstacles_lock:
                        tentative_ids = {batch.known_id for batch in tentative_sx_batches_for_obstacles}
                        self.obstacles = [
                            obs for obs in self.obstacles
                            if not any(obs.id.startswith(f"sx_{bid}_") for bid in tentative_ids)
                        ]
                        self.obstacle_sources = {
                            k: v for k, v in self.obstacle_sources.items()
                            if not any(k.startswith(f"sx_{bid}_") for bid in tentative_ids)
                        }
                    sx_tentative_state = ArucoMapperState()
                    sx_tentative_state.stamp = response.stable_state.stamp
                    sx_tentative_state.initialized = response.stable_state.initialized
                    sx_tentative_state.batches = tentative_sx_batches_for_obstacles
                    self._handle_aruco_mapper_state(sx_tentative_state, 'sx')

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

                if fx_batches_ready_for_obstacles:
                    with self.batches_lock:
                        for batch in fx_batches_ready_for_obstacles:
                            self.batches_with_obstacles.discard(batch.known_id)
                    with self.obstacles_lock:
                        fx_ready_ids = {batch.known_id for batch in fx_batches_ready_for_obstacles}
                        self.obstacles = [
                            obs for obs in self.obstacles
                            if not any(obs.id.startswith(f"fx_{bid}_") for bid in fx_ready_ids)
                        ]
                        self.obstacle_sources = {
                            k: v for k, v in self.obstacle_sources.items()
                            if not any(k.startswith(f"fx_{bid}_") for bid in fx_ready_ids)
                        }
                        for bid in fx_ready_ids:
                            self.fx_safety_added.discard(f"fx_{bid}_safety")
                    fx_lock_state = ArucoMapperState()
                    fx_lock_state.stamp = response.stable_state.stamp
                    fx_lock_state.initialized = response.stable_state.initialized
                    fx_lock_state.batches = fx_batches_ready_for_obstacles
                    self._handle_aruco_mapper_state(fx_lock_state, 'fx')

                if tentative_fx_batches_for_obstacles:
                    with self.obstacles_lock:
                        tentative_fx_ids = {batch.known_id for batch in tentative_fx_batches_for_obstacles}
                        self.obstacles = [
                            obs for obs in self.obstacles
                            if not any(obs.id.startswith(f"fx_{bid}_") for bid in tentative_fx_ids)
                        ]
                        self.obstacle_sources = {
                            k: v for k, v in self.obstacle_sources.items()
                            if not any(k.startswith(f"fx_{bid}_") for bid in tentative_fx_ids)
                        }
                        for bid in tentative_fx_ids:
                            self.fx_safety_added.discard(f"fx_{bid}_safety")
                    fx_tentative_state = ArucoMapperState()
                    fx_tentative_state.stamp = response.stable_state.stamp
                    fx_tentative_state.initialized = response.stable_state.initialized
                    fx_tentative_state.batches = tentative_fx_batches_for_obstacles
                    self._handle_aruco_mapper_state(fx_tentative_state, 'fx')
                    self._start_short_pattern_retry_timer()
            else:
                self.node.get_logger().error("❌ Failed to lock baseline patterns: no response")
        except Exception as e:
            self.node.get_logger().error(f"❌ Error locking baseline patterns: {e}")

    # ------------------------------------------------------------------
    # Post-lock /aruco_mapper/all_batches updates
    # ------------------------------------------------------------------

    def _handle_all_batches_topic(self, msg: ArucoMapperState) -> None:
        """Handle incremental batch updates after baseline lock is stable."""
        if self.game_timeout_stopped:
            return
        if not self.all_batches_sub_active:
            return

        if not self.baselines_locked:
            # Allow P/NP status refresh before baseline lock is stable.
            for batch in (msg.batches or []):
                known_id = str(getattr(batch, 'known_id', '') or '').strip()
                if not known_id or known_id.startswith('g'):
                    continue

                status_suffix = self._parse_batch_state(batch.state)[1]
                with self.batches_lock:
                    if known_id.startswith('s'):
                        if status_suffix in ('P', 'NP'):
                            self.batches_sx_status_type[known_id] = status_suffix
                        else:
                            self.batches_sx_status_type.pop(known_id, None)
                    else:
                        if status_suffix in ('P', 'NP'):
                            self.batches_fx_status_type[known_id] = status_suffix
                        else:
                            self.batches_fx_status_type.pop(known_id, None)
            return

        # Remove LOST batches immediately.
        lost_ids = {
            batch.known_id
            for batch in (msg.batches or [])
            if self._parse_batch_state(batch.state)[0] == 'LOST'
        }
        # Ignore GX in topic-driven updates.
        lost_ids = {bid for bid in lost_ids if not str(bid or '').startswith('g')}
        if lost_ids:
            with self.batches_lock:
                manual_putdown_ids = set(self.manual_putdown_batch_ids)
            lost_non_manual_ids = lost_ids - manual_putdown_ids
            if lost_ids - lost_non_manual_ids:
                self.node.get_logger().info(
                    f"📌 Ignoring LOST for manual put_down batches (preserved): {lost_ids - lost_non_manual_ids}"
                )
            self._remove_lost_batches(lost_non_manual_ids, reason="reported LOST by /aruco_mapper/all_batches")
            for bid in lost_ids:
                self.pending_baseline_request_ids.discard(bid)
                self.fx_pending_baseline_batches.pop(bid, None)

        # For new batches, request locked baseline to get final pattern.
        for batch in (msg.batches or []):
            known_id = str(getattr(batch, 'known_id', '') or '').strip()
            if not known_id:
                continue

            state, _ = self._parse_batch_state(batch.state)
            if state == 'LOST':
                continue

            if known_id in self.deleted_batch_ids:
                continue

            if known_id.startswith('g'):
                continue

            is_sx = known_id.startswith('s')
            is_fx = known_id.startswith('f') or not (known_id.startswith('s') or known_id.startswith('g'))

            # Always refresh P/NP quality status from /aruco_mapper/all_batches.
            status_suffix = self._parse_batch_state(batch.state)[1]
            with self.batches_lock:
                if is_sx:
                    if status_suffix in ('P', 'NP'):
                        self.batches_sx_status_type[known_id] = status_suffix
                    else:
                        self.batches_sx_status_type.pop(known_id, None)
                elif is_fx:
                    if status_suffix in ('P', 'NP'):
                        self.batches_fx_status_type[known_id] = status_suffix
                    else:
                        self.batches_fx_status_type.pop(known_id, None)

            if is_sx:
                # New SX after lock: request a locked baseline to get pattern.
                if known_id not in self.locked_baseline_all_batch_ids:
                    self._request_locked_baseline_for_sx(known_id)
                continue

            if is_fx:
                with self.batches_lock:
                    if known_id in self.manual_putdown_batch_ids:
                        continue

                member_count = len(batch.members) if getattr(batch, 'members', None) else 0

                with self.batches_lock:
                    existing_batch = self.batches_fx.get(known_id)
                    existing_member_count = (
                        len(existing_batch.members)
                        if existing_batch and getattr(existing_batch, 'members', None)
                        else 0
                    )
                    already_known = existing_batch is not None

                member_count_grew = member_count > existing_member_count

                # Skip if nothing new to learn (same or fewer members).
                if (already_known or known_id in self.pending_baseline_request_ids) and not member_count_grew:
                    continue

                # Save as temporary and request locked baseline to conclude pattern.
                with self.batches_lock:
                    self.batches_fx[known_id] = batch

                self.fx_pending_baseline_batches[known_id] = batch

                # Add FX obstacles immediately (even if EMERGING) when members exist.
                if member_count > 0:
                    # If member count grew, tear down the old obstacles so the guard
                    # in _handle_aruco_mapper_state doesn't skip the rebuild.
                    if member_count_grew and known_id in self.batches_with_obstacles:
                        with self.obstacles_lock:
                            old_obs = [o for o in self.obstacles if f"fx_{known_id}_member" in o.id]
                            for o in old_obs:
                                self.obstacles.remove(o)
                                self.obstacle_sources.pop(o.id, None)
                        self.batches_with_obstacles.discard(known_id)
                        self.node.get_logger().info(
                            f"🔄 FX '{known_id}' grew {existing_member_count}→{member_count} members, rebuilding obstacles"
                        )
                    fx_state = ArucoMapperState()
                    fx_state.stamp = msg.stamp
                    fx_state.initialized = msg.initialized
                    fx_state.batches = [batch]
                    self._handle_aruco_mapper_state(fx_state, 'fx')

                # Only request a new locked baseline if one isn't already in-flight.
                if known_id not in self.pending_baseline_request_ids:
                    self._request_locked_baseline_for_known_id(known_id)

    def _request_locked_baseline_for_known_id(self, known_id: str) -> bool:
        """Request locked baseline for a single batch (FX or GX), SX uses its own path."""
        if not known_id:
            return False

        if known_id.startswith('s'):
            return self._request_locked_baseline_for_sx(known_id)

        if known_id in self.pending_baseline_request_ids:
            return True

        now_s = time.monotonic()
        last_req_s = float(self._last_locked_baseline_request_s.get(known_id, 0.0))
        if now_s - last_req_s < max(self.baseline_request_cooldown_sec, 0.0):
            return False

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
            return False

        self.pending_baseline_request_ids.add(known_id)
        self._last_locked_baseline_request_s[known_id] = now_s
        self.node.get_logger().info(
            f"🔒 Requesting locked baseline for batch '{known_id}'"
        )

        req = GetStableBatches.Request()
        req.batch_types_filter = "sfg"
        req.specific_known_id = known_id
        req.only_locked_baselines = True
        future = client.call_async(req)
        future.add_done_callback(
            lambda f, kid=known_id, c=client, s=self.retry_service_name: self._handle_locked_baseline_from_topic(f, kid, c, s)
        )
        return True

    def _handle_locked_baseline_from_topic(self, future, known_id: str, client, service_name: str) -> None:
        """Handle locked-baseline response requested from /aruco_mapper/all_batches."""
        try:
            self._handle_get_stable_batches_response(
                future,
                None,
                client,
                service_name,
                known_id,
                False,
                False,
            )
        finally:
            self.pending_baseline_request_ids.discard(known_id)
            self.fx_pending_baseline_batches.pop(known_id, None)

    def _retry_timer_callback(self):
        """Timer callback to retry EMERGING SX known_ids only."""
        if self.game_timeout_stopped:
            return
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
                    f, None, self.retry_client, self.retry_service_name, kid, True, True
                )
            )
    
    # ------------------------------------------------------------------
    # Short-pattern retry helpers
    # ------------------------------------------------------------------

    def _start_short_pattern_retry_timer(self):
        """Start a periodic timer that re-checks locked baselines for batches
        whose pattern currently has fewer than 4 colour elements, or whose
        locked baseline status is tentative (T)."""
        if self.short_pattern_retry_active:
            return

        self.short_pattern_retry_active = True
        interval = max(self.short_pattern_retry_interval_sec, 0.5)
        total_watch = len(self.short_pattern_ids) + len(self.tentative_baseline_ids)
        self.node.get_logger().info(
            f"🔍 Short-pattern retry enabled: re-checking every {interval:.1f}s "
            f"for {total_watch} batches (short={self.short_pattern_ids}, tentative={self.tentative_baseline_ids})"
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
        if self.game_timeout_stopped:
            return
        if not self.short_pattern_retry_active:
            return

        if not self.short_pattern_ids and not self.tentative_baseline_ids:
            self._stop_short_pattern_retry_timer()
            return

        if not self.retry_client or not self.retry_client.service_is_ready():
            self.node.get_logger().warn("⚠️ Short-pattern retry: batch service not ready")
            return

        self.node.get_logger().info(
            f"🔍 Re-checking locked-baseline batches: short={self.short_pattern_ids}, tentative={self.tentative_baseline_ids}"
        )
        for known_id in list(self.short_pattern_ids | self.tentative_baseline_ids):
            is_sx = known_id.startswith('s')
            is_fx = not is_sx and not known_id.startswith('g')
            if not is_sx and not is_fx:
                self.short_pattern_ids.discard(known_id)
                self.tentative_baseline_ids.discard(known_id)
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

                self._apply_manual_pattern_override(batch)
                new_pattern = batch.pattern if batch.pattern else ""
                _parsed_state, baseline_status = self._parse_batch_state(batch.state)

                is_sx = known_id.startswith('s')
                is_fx = not is_sx and not known_id.startswith('g')

                if _parsed_state == 'LOST':
                    self.node.get_logger().info(
                        f"🗑️  Short-pattern re-check for '{known_id}': LOST → stopping retry"
                    )
                    self.short_pattern_ids.discard(known_id)
                    self.tentative_baseline_ids.discard(known_id)
                    if not self.short_pattern_ids and not self.tentative_baseline_ids:
                        self._stop_short_pattern_retry_timer()
                    break

                if is_fx:
                    with self.batches_lock:
                        is_putdown = known_id in self.manual_putdown_batch_ids
                    if is_putdown:
                        self.tentative_baseline_ids.discard(known_id)
                        self.short_pattern_ids.discard(known_id)
                        self.node.get_logger().info(
                            f"📌 Short-pattern retry: skipping '{known_id}' — manual put_down batch, not overwriting"
                        )
                        break
                    with self.batches_lock:
                        self.batches_fx[known_id] = batch
                        _, fx_status_type = self._parse_batch_state(batch.state)
                        if fx_status_type:
                            self.batches_fx_status_type[known_id] = fx_status_type
                        else:
                            self.batches_fx_status_type.pop(known_id, None)
                        self.batches_with_obstacles.discard(known_id)

                    member_prefix = f"fx_{known_id}_member"
                    safety_id = f"fx_{known_id}_safety"

                    if _parsed_state == 'BASELINE_TEMPORARY':
                        # Tentative: only refresh member obstacles, keep safety zone in place
                        with self.obstacles_lock:
                            self.obstacles = [obs for obs in self.obstacles if not obs.id.startswith(member_prefix)]
                            self.obstacle_sources = {k: v for k, v in self.obstacle_sources.items() if not k.startswith(member_prefix)}
                        self.tentative_baseline_ids.add(known_id)
                        self.locked_baseline_batch_ids.discard(known_id)
                        self.locked_baseline_all_batch_ids.discard(known_id)
                        self.node.get_logger().info(
                            f"[TEMP_BASELINE] ⏳ FX '{known_id}': still BASELINE_TEMPORARY, pattern='{new_pattern}' → retrying"
                        )
                    else:
                        # Confirmed: clear everything including safety zone for clean rebuild
                        with self.obstacles_lock:
                            self.obstacles = [obs for obs in self.obstacles if not obs.id.startswith(member_prefix) and obs.id != safety_id]
                            self.obstacle_sources = {k: v for k, v in self.obstacle_sources.items() if not k.startswith(member_prefix) and k != safety_id}
                            self.fx_safety_added.discard(safety_id)
                        self.tentative_baseline_ids.discard(known_id)
                        self.short_pattern_ids.discard(known_id)
                        self.locked_baseline_batch_ids.add(known_id)
                        self.locked_baseline_all_batch_ids.add(known_id)
                        self.node.get_logger().info(
                            f"[FINAL_BASELINE] ✅ FX '{known_id}': baseline confirmed, pattern='{new_pattern}'"
                        )

                    if batch.members:
                        fx_retry_state = ArucoMapperState()
                        fx_retry_state.stamp = response.stable_state.stamp
                        fx_retry_state.initialized = response.stable_state.initialized
                        fx_retry_state.batches = [batch]
                        self._handle_aruco_mapper_state(fx_retry_state, 'fx')

                    if not self.short_pattern_ids and not self.tentative_baseline_ids:
                        self._stop_short_pattern_retry_timer()
                        self._try_publish_beacon_ready()
                    break

                with self.batches_lock:
                    existing = self.batches_sx.get(known_id) or self.batches_fx.get(known_id)
                    old_pattern = existing.pattern if (existing and existing.pattern) else ""

                already_locked = (
                    known_id in self.locked_baseline_batch_ids
                    or known_id in self.locked_baseline_all_batch_ids
                )

                if baseline_status == 'T':
                    if already_locked:
                        self.short_pattern_ids.discard(known_id)
                        self.tentative_baseline_ids.discard(known_id)
                        self.node.get_logger().info(
                            f"[STICKY_BASELINE] ℹ️  Locked SX '{known_id}' stayed tentative (T); keeping existing lock until LOST or put_down"
                        )
                        if not self.short_pattern_ids and not self.tentative_baseline_ids:
                            self._stop_short_pattern_retry_timer()
                        break

                    with self.batches_lock:
                        self.batches_sx[known_id] = batch
                        self.batches_sx_baseline_status_type[known_id] = 'T'
                        _, sx_status_type = self._parse_batch_state(batch.state)
                        if sx_status_type in ('P', 'NP'):
                            self.batches_sx_status_type[known_id] = sx_status_type
                        else:
                            self.batches_sx_status_type.pop(known_id, None)
                        self.batches_with_obstacles.discard(known_id)
                    self.locked_baseline_batch_ids.discard(known_id)
                    self.locked_baseline_all_batch_ids.discard(known_id)
                    self.tentative_baseline_ids.add(known_id)

                    prefix_sx = f"sx_{known_id}_"
                    with self.obstacles_lock:
                        self.obstacles = [obs for obs in self.obstacles if not obs.id.startswith(prefix_sx)]
                        self.obstacle_sources = {
                            k: v for k, v in self.obstacle_sources.items() if not k.startswith(prefix_sx)
                        }

                    self.node.get_logger().info(
                        f"[TEMP_BASELINE] ⏳ Locked baseline still tentative for '{known_id}' (status=T), retrying"
                    )

                    # Keep temporary SX obstacles visible and color-synced while
                    # waiting for constrained baseline confirmation.
                    sx_state = ArucoMapperState()
                    sx_state.stamp = response.stable_state.stamp
                    sx_state.initialized = response.stable_state.initialized
                    sx_state.batches = [batch]
                    self._handle_aruco_mapper_state(sx_state, 'sx')
                    break

                if baseline_status == 'C':
                    self.batches_sx_baseline_status_type[known_id] = 'C'
                    self.tentative_baseline_ids.discard(known_id)

                if len(new_pattern) >= 4 or baseline_status == 'C':
                    self.node.get_logger().info(
                        f"[FINAL_BASELINE] ✅ Short-pattern resolved for '{known_id}': "
                        f"'{old_pattern}' → '{new_pattern}' ({len(new_pattern)} elements)"
                    )

                    self.locked_baseline_batch_ids.add(known_id)
                    self.locked_baseline_all_batch_ids.add(known_id)

                    with self.batches_lock:
                        self.batches_sx[known_id] = batch
                        _, sx_status_type = self._parse_batch_state(batch.state)
                        if sx_status_type in ('P', 'NP'):
                            self.batches_sx_status_type[known_id] = sx_status_type
                        else:
                            self.batches_sx_status_type.pop(known_id, None)
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
                    if not self.short_pattern_ids and not self.tentative_baseline_ids:
                        self._stop_short_pattern_retry_timer()

                    self._try_publish_beacon_ready()

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
                self._apply_manual_pattern_override(batch)
                if source == 'gx':
                    continue

                if source == 'fx':
                    with self.batches_lock:
                        if batch.known_id in self.manual_putdown_batch_ids:
                            self.node.get_logger().debug(
                                f"📌 Keeping manual put_down pose for '{batch.known_id}' (ignore non-LOST beacon update)"
                            )
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
                        _, status_type = self._parse_batch_state(batch.state)
                        if status_type:
                            self.batches_fx_status_type[batch.known_id] = status_type
                        else:
                            self.batches_fx_status_type.pop(batch.known_id, None)
                    elif source == 'sx':
                        self.batches_sx[batch.known_id] = batch
                        _, sx_status_type = self._parse_batch_state(batch.state)
                        if sx_status_type in ('P', 'NP'):
                            self.batches_sx_status_type[batch.known_id] = sx_status_type
                        else:
                            self.batches_sx_status_type.pop(batch.known_id, None)

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

                    is_tentative_baseline = batch_known_id in self.tentative_baseline_ids

                    if batch_known_id not in self.locked_baseline_batch_ids and not is_tentative_baseline:
                        self._request_locked_baseline_for_sx(batch_known_id)
                        self.node.get_logger().debug(
                            f"⏳ Deferring SX obstacles for '{batch_known_id}' until its baseline is locked"
                        )
                        continue

                    pattern_colors = self.parse_pattern_colors((batch.pattern or "").lower())
                    member_colors = []
                    if batch.members:
                        for member in batch.members:
                            member_color = str(getattr(member, 'color', '') or '').strip().lower()
                            if member_color in ('blue', 'yellow'):
                                member_colors.append(member_color)

                    if len(pattern_colors) < 4 and len(member_colors) >= 1:
                        # For tentative baselines, prefer what is currently observed.
                        # For confirmed baselines, this still acts as a safe fallback.
                        pattern_colors = member_colors[:4]

                    if len(pattern_colors) < 4 and not is_tentative_baseline:
                        _parsed_state, baseline_status = self._parse_batch_state(batch.state)
                        if baseline_status == 'C' and batch.members and len(batch.members) >= 4:
                            fallback_colors = member_colors[:4]
                            if len(fallback_colors) == 4:
                                pattern_colors = fallback_colors
                            else:
                                self.node.get_logger().warn(
                                    f"⚠️  Locked SX batch '{batch_known_id}' status=C but pattern/member colors incomplete, waiting before obstacle creation"
                                )
                                continue
                        else:
                            self.node.get_logger().warn(
                                f"⚠️  Locked SX batch '{batch_known_id}' still has incomplete pattern '{batch.pattern}', waiting before obstacle creation"
                            )
                            continue

                    member_count = len(batch.members) if batch.members else 0
                    if member_count == 3 and not is_tentative_baseline:
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

                    # Tentative baselines are rendered with currently known crates,
                    # then replaced when confirmed baseline data arrives.
                    if is_tentative_baseline:
                        # If a full temporary pattern exists (e.g. "yybb"), draw all
                        # 4 crates from that pattern even when member detection is partial.
                        if len(pattern_colors) == 4:
                            crates_to_draw = 4
                        else:
                            tentative_count = member_count if member_count > 0 else len(pattern_colors)
                            crates_to_draw = max(1, min(4, tentative_count, len(pattern_colors)))
                    else:
                        crates_to_draw = 4

                    for idx, (offset_x, offset_y) in enumerate(offsets[:crates_to_draw]):
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

                # If this is an FX batch, ensure we also add a 170x170 safety-zone
                # centered on the reported batch center (one per FX known_id).
                if source == 'fx':
                    safety_id = f"fx_{batch.known_id}_safety"
                    # If we've already added a safety obstacle for this FX known_id, skip re-adding.
                    if safety_id in self.fx_safety_added:
                        self.node.get_logger().debug(f"Skipping duplicate safety zone for {batch.known_id} (already added)")
                    else:
                        # Also be conservative: avoid adding if an obstacle with the same id already exists
                        if safety_id in self.obstacle_sources:
                            self.fx_safety_added.add(safety_id)
                            self.node.get_logger().debug(f"Found existing obstacle source for {safety_id}, marking as added")
                        else:
                            try:
                                safety_zone_size_mm = 170.0
                                center_x_mm = float(batch.center.x) * 1000.0 if hasattr(batch, 'center') else None
                                center_y_mm = float(batch.center.y) * 1000.0 if hasattr(batch, 'center') else None
                            except Exception:
                                center_x_mm = None
                                center_y_mm = None

                            # Fallback to average of member positions if batch.center missing
                            if center_x_mm is None or center_y_mm is None:
                                # compute average from members
                                try:
                                    mx = sum(m.map_pos.x for m in batch.members) / len(batch.members)
                                    my = sum(m.map_pos.y for m in batch.members) / len(batch.members)
                                    center_x_mm = mx * 1000.0
                                    center_y_mm = my * 1000.0
                                except Exception:
                                    center_x_mm = 0.0
                                    center_y_mm = 0.0

                            safety_obs = Obstacle()
                            safety_obs.id = safety_id
                            safety_obs.type = 'safety_zone'
                            safety_obs.color = 'wall'
                            safety_obs.width_mm = safety_zone_size_mm
                            safety_obs.height_mm = safety_zone_size_mm
                            safety_obs.pose.x = center_x_mm
                            safety_obs.pose.y = center_y_mm
                            safety_obs.pose.theta = 0.0
                            safety_obs.detected_at = msg.stamp
                            self.obstacles.append(safety_obs)
                            self.obstacle_sources[safety_obs.id] = 'beacon_safety'
                            self.fx_safety_added.add(safety_id)
                            self.node.get_logger().info(
                                f"🛡️ Added safety zone for {batch.known_id}: {safety_zone_size_mm:.0f}mm x {safety_zone_size_mm:.0f}mm at ({center_x_mm:.1f}, {center_y_mm:.1f})mm"
                            )

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
                self.batches_fx_status_type.pop(known_id, None)
                deleted_from_fx = True
                self.node.get_logger().info(f"🗑️  Deleted batch '{known_id}' from FX storage")
            
            if known_id in self.batches_sx:
                del self.batches_sx[known_id]
                self.batches_sx_status_type.pop(known_id, None)
                deleted_from_sx = True
                self.node.get_logger().info(f"🗑️  Deleted batch '{known_id}' from SX storage")
            
            # Remove from obstacle creation tracking
            self.batches_with_obstacles.discard(known_id)
            self.locked_baseline_batch_ids.discard(known_id)
            self.locked_baseline_all_batch_ids.discard(known_id)
            self.pending_baseline_lock_ids.discard(known_id)
            self.short_pattern_ids.discard(known_id)
            self.tentative_baseline_ids.discard(known_id)
            self.manual_putdown_batch_ids.discard(known_id)
            self.synthetic_override_batch_ids.discard(known_id)
            self.active_batch_ids.discard(known_id)
            self.known_ids_to_retry.discard(known_id)
            self.batches_sx_baseline_status_type.pop(known_id, None)
            self.batches_sx_status_type.pop(known_id, None)

            # Persist manual deletion so beacon updates do not re-add this batch
            # (and therefore do not trigger new baseline-lock requests).
            self.deleted_batch_ids.add(known_id)
            
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

        self.node.get_logger().info(
            f"🚫 Batch '{known_id}' marked deleted (will be ignored in updates): {self.deleted_batch_ids}"
        )
        
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
            - qualities: List of quality tags (FX: 'P'/'NP' when available, otherwise '')
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
                    qualities.append(str(self.batches_fx_status_type.get(known_id, '')))
            
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
                    qualities.append(str(self.batches_sx_status_type.get(known_id, '')))

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
