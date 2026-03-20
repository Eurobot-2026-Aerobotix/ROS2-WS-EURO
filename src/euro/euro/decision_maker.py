#!/usr/bin/env python3
"""
Decision Maker Module for Dynamic Batch Stealing

This module generates dynamic tasks to steal batches when:
- Path generation is impossible
- Someone steals a batch we're going to pick up
- Static batches are finished

Features:
- Tracks front/back picker status (full/empty)
- Maintains predefined pickup poses for SX batches (S1-S8)
- Queries available batches from map.py via pathfinder
- Generates complete tasks with STM32 waypoints and ESP32 commands
"""

import math
import threading
from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
from geometry_msgs.msg import Point
import time


def circular_angle_difference(angle1: float, angle2: float) -> float:
    """
    Calculate the minimum angular difference between two angles in degrees.
    Handles wrap-around at ±180°.
    
    Args:
        angle1: First angle in degrees
        angle2: Second angle in degrees
    
    Returns:
        Minimum absolute difference in degrees (0-180)
    """
    diff = abs(angle1 - angle2)
    if diff > 180:
        diff = 360 - diff
    return diff


# Predefined pickup poses for SX batches (positions in mm)
PICKUP_POSES = {
    'S1': [{'x': 800, 'y': 400, 'theta': [90, -90]}],
    'S2': [{'x': 1600, 'y': 400, 'theta': [90, -90]}],
    'S3': [
        {'x': 1430, 'y': 1150, 'theta': [0, 180]},
        {'x': 970, 'y': 1150, 'theta': [0, 180]}
    ],
    'S4': [{'x': 1600, 'y': 1100, 'theta': [0, 180]}],
    'S5': [
        {'x': 1430, 'y': 1850, 'theta': [0, 180]},
        {'x': 970, 'y': 1850, 'theta': [0, 180]}
    ],
    'S6': [{'x': 1600, 'y': 1900, 'theta': [0, 180]}],
    'S7': [{'x': 1600, 'y': 2600, 'theta': [0, 180]}],
    'S8': [{'x': 800, 'y': 2600, 'theta': [0, 180]}]
}

# Predefined putdown poses for FX zones (positions in mm)
PUTDOWN_POSES = {
    'fx1': {'x': 1200, 'y': 100, 'type': 'A'},
    'fx2': {'x': 1900, 'y': 700, 'type': 'A'},
    'fx3': {'x': 1200, 'y': 800, 'type': 'B'},
    'fx5': {'x': 1200, 'y': 1500, 'type': 'B'},
    'fx6': {'x': 1900, 'y': 1500, 'type': 'A'},
    'fx8': {'x': 1200, 'y': 2200, 'type': 'B'},
    'fx9': {'x': 1900, 'y': 2300, 'type': 'A'},
    'fx10': {'x': 1200, 'y': 2900, 'type': 'A'}
}


@dataclass(frozen=True)
class BatchRecord:
    """Immutable batch entry stored in the registry."""
    known_id: str
    available: bool
    color: Optional[str]
    batch_type: str
    pickup_poses: Tuple[Dict[str, Any], ...]
    putdown_pose: Optional[Dict[str, Any]]
    fx_subtype: Optional[str]
    last_seen: float
    confidence: float
    version: int
    member_count: int
    direction: str
    center: Point
    quality: float


class BatchRegistry:
    """
    Thread-safe single source of truth for active batches.

    Provides immutable snapshot reads (deep-copied dicts) so decision logic can
    safely consume data without holding locks.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._records: Dict[str, BatchRecord] = {}
        self._version: int = 0
        self._stale: bool = False
        self._last_error: Optional[str] = None
        self._last_success_update: float = 0.0

    @staticmethod
    def _clone_point(point: Point) -> Point:
        p = Point()
        p.x = point.x
        p.y = point.y
        p.z = point.z
        return p

    @staticmethod
    def _normalize_base_known_id(known_id: str) -> str:
        return known_id.upper().replace('_WEB', '')

    @staticmethod
    def _normalize_fx_zone_key(known_id: str) -> Optional[str]:
        base = known_id.lower().replace('_web', '')
        if base.startswith('fx'):
            return base
        if base.startswith('f') and base[1:].isdigit():
            return f'fx{base[1:]}'
        return None

    @staticmethod
    def _safe_ratio(value: float, max_value: float) -> float:
        if max_value <= 0:
            return 0.0
        return max(0.0, min(1.0, value / max_value))

    def _resolve_pickup_poses(self, known_id: str, batch_type: str) -> Tuple[Dict[str, Any], ...]:
        if batch_type != 'sx':
            return tuple()
        base_id = self._normalize_base_known_id(known_id)
        poses = PICKUP_POSES.get(base_id, [])
        return tuple(deepcopy(poses))

    def _resolve_putdown_pose_and_fx_subtype(self, known_id: str, batch_type: str) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
        if batch_type != 'fx':
            return None, None
        zone_key = self._normalize_fx_zone_key(known_id)
        if zone_key is None:
            return None, None
        pose = PUTDOWN_POSES.get(zone_key)
        if pose is None:
            return None, None
        pose_copy = deepcopy(pose)
        return pose_copy, pose_copy.get('type')

    def update_from_batch_info(self, batch_info: Dict[str, List[Any]]) -> int:
        """Replace current registry content from pathfinder active batch arrays."""
        required = ['known_ids', 'batch_types', 'member_counts', 'directions', 'centers', 'qualities']
        for key in required:
            if key not in batch_info:
                raise KeyError(f'Missing key "{key}" in batch_info')

        known_ids = batch_info['known_ids']
        batch_types = batch_info['batch_types']
        member_counts = batch_info['member_counts']
        directions = batch_info['directions']
        centers = batch_info['centers']
        qualities = batch_info['qualities']

        n = min(
            len(known_ids),
            len(batch_types),
            len(member_counts),
            len(directions),
            len(centers),
            len(qualities)
        )

        now = time.time()

        with self._lock:
            self._version += 1
            version = self._version
            next_records: Dict[str, BatchRecord] = {}

            for i in range(n):
                known_id = str(known_ids[i])
                batch_type = str(batch_types[i]).lower()
                member_count = int(member_counts[i])
                direction = str(directions[i])
                center = self._clone_point(centers[i])
                quality = float(qualities[i])
                confidence = self._safe_ratio(quality, 100.0)

                pickup_poses = self._resolve_pickup_poses(known_id, batch_type)
                putdown_pose, fx_subtype = self._resolve_putdown_pose_and_fx_subtype(known_id, batch_type)

                next_records[known_id] = BatchRecord(
                    known_id=known_id,
                    available=member_count > 0,
                    color=None,
                    batch_type=batch_type,
                    pickup_poses=pickup_poses,
                    putdown_pose=putdown_pose,
                    fx_subtype=fx_subtype,
                    last_seen=now,
                    confidence=confidence,
                    version=version,
                    member_count=member_count,
                    direction=direction,
                    center=center,
                    quality=quality,
                )

            self._records = next_records
            self._stale = False
            self._last_error = None
            self._last_success_update = now

            return len(next_records)

    def mark_stale(self, error_message: str):
        with self._lock:
            self._stale = True
            self._last_error = error_message

    def get_snapshot(self) -> Dict[str, Dict[str, Any]]:
        """Return deep-copied immutable snapshot data for decision logic reads."""
        with self._lock:
            records = list(self._records.values())

        snapshot: Dict[str, Dict[str, Any]] = {}
        for record in records:
            snapshot[record.known_id] = {
                'known_id': record.known_id,
                'available': record.available,
                'color': record.color,
                'batch_type': record.batch_type,
                'pickup_poses': [deepcopy(p) for p in record.pickup_poses],
                'putdown_pose': deepcopy(record.putdown_pose) if record.putdown_pose is not None else None,
                'fx_subtype': record.fx_subtype,
                'last_seen': record.last_seen,
                'confidence': record.confidence,
                'version': record.version,
                # Backward-compat fields used by existing decision logic
                'member_count': record.member_count,
                'direction': record.direction,
                'center': self._clone_point(record.center),
                'quality': record.quality,
            }
        return snapshot

    def metadata(self) -> Dict[str, Any]:
        with self._lock:
            return {
                'version': self._version,
                'stale': self._stale,
                'last_error': self._last_error,
                'last_success_update': self._last_success_update,
                'size': len(self._records),
            }


class MapUpdater:
    """Background updater that refreshes map + active batches at fixed frequency."""

    def __init__(self, strategy_manager, pathfinder_client, batch_registry: BatchRegistry, update_hz: float = 4.0):
        self.strategy = strategy_manager
        self.pathfinder = pathfinder_client
        self.registry = batch_registry
        self.update_hz = max(2.0, min(5.0, float(update_hz)))

        self._stop_event = threading.Event()
        self._update_lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

        self._enabled = True
        self._last_state_log_s = 0.0
        self._warned_missing_update_map = False

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name='decision-maker-map-updater', daemon=True)
        self._thread.start()
        self.strategy.get_logger().info(
            f'🛰️ MapUpdater started in background ({self.update_hz:.1f}Hz)'
        )

    def stop(self):
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)

    def set_enabled(self, enabled: bool):
        self._enabled = enabled

    def _run(self):
        period_s = 1.0 / self.update_hz
        while not self._stop_event.is_set():
            loop_start = time.perf_counter()
            if self._enabled:
                self.update_once()
            elapsed = time.perf_counter() - loop_start
            sleep_s = max(0.0, period_s - elapsed)
            self._stop_event.wait(timeout=sleep_s)

    def update_once(self):
        """Single non-overlapping update attempt; keeps last good snapshot on failures."""
        if not self._update_lock.acquire(blocking=False):
            return

        try:
            # Some adapters (e.g. web PathfinderAdapter) do not expose update_map().
            # In that case, rely on get_active_batches() only.
            update_map_fn = getattr(self.pathfinder, 'update_map', None)
            if callable(update_map_fn):
                map_success, map_message = update_map_fn()
                if not map_success:
                    self.registry.mark_stale(f'update_map failed: {map_message}')
                    self.strategy.get_logger().warning(f'⚠️ Background update_map failed: {map_message}')
                    return
            elif not self._warned_missing_update_map:
                self.strategy.get_logger().warning(
                    '⚠️ Pathfinder has no update_map(); background updater will refresh via get_active_batches only'
                )
                self._warned_missing_update_map = True

            batch_info = self.pathfinder.get_active_batches()
            if batch_info is None:
                self.registry.mark_stale('get_active_batches returned None')
                self.strategy.get_logger().warning('⚠️ Background get_active_batches failed')
                return

            updated_count = self.registry.update_from_batch_info(batch_info)

            now = time.time()
            if now - self._last_state_log_s >= 2.0:
                meta = self.registry.metadata()
                self.strategy.get_logger().info(
                    f'🧠 Registry sync: {updated_count} batches (version={meta["version"]}, stale={meta["stale"]})'
                )
                self._last_state_log_s = now

        except Exception as exc:
            self.registry.mark_stale(f'exception: {exc}')
            self.strategy.get_logger().error(f'❌ Background map updater exception: {exc}')
        finally:
            self._update_lock.release()


class DecisionMaker:
    """
    Decision maker that generates dynamic tasks to steal batches
    """
    
    def __init__(self, strategy_manager, pathfinder_client):
        """
        Initialize the decision maker
        
        Args:
            strategy_manager: StrategyManager instance (for logging and state access)
            pathfinder_client: PathfinderClient instance to query batches and generate paths
        """
        self.strategy = strategy_manager
        self.pathfinder = pathfinder_client
        
        # Track picker status (True = full, False = empty)
        self.front_full = False
        self.back_full = False
        
        # Cache for available batches
        self.available_batches = {}
        self.last_batch_update = 0.0
        self._last_registry_version = -1
        self._last_batch_sync_log_s = 0.0
        self.batch_registry = BatchRegistry()
        self.map_updater = MapUpdater(
            strategy_manager=self.strategy,
            pathfinder_client=self.pathfinder,
            batch_registry=self.batch_registry,
            update_hz=4.0,
        )
        self.map_updater.set_enabled(True)
        self.map_updater.start()

        # Enemy proximity thresholds (in mm)
        # - hard_block_radius_mm: batch is rejected if enemy is this close
        # - near_penalty_radius_mm: batch score is penalized if enemy is within this radius
        self.hard_block_radius_mm = 280.0
        self.near_penalty_radius_mm = 520.0

        self._update_map_timer = self.strategy.create_timer(1.0, self._update_map_during_round)
        
        self.strategy.get_logger().info('🧠 Decision Maker initialized')

    def __del__(self):
        try:
            if hasattr(self, 'map_updater') and self.map_updater is not None:
                self.map_updater.stop()
        except Exception:
            pass

    def _log_duration_ms(self, label: str, start_time: float):
        """Log elapsed duration in milliseconds for profiling decision flow."""
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        self.strategy.get_logger().info(f'⏱️ {label}: {elapsed_ms:.1f}ms')

    def _update_map_during_round(self):
        """Keep background updater enabled; optionally stop after round timeout if available."""
        round_timer = getattr(self.strategy, 'round', None)
        if round_timer is None:
            # Web/simulation mode often has no round timer: keep updater active.
            self.map_updater.set_enabled(True)
            return

        if not round_timer.has_started():
            self.map_updater.set_enabled(True)
            return

        elapsed = round_timer.time()
        if elapsed >= 100.0:
            # Preserve previous behavior near end of match.
            self.map_updater.set_enabled(False)
            return

        self.map_updater.set_enabled(True)

    def _get_enemy_position_mm(self) -> Optional[Tuple[float, float]]:
        """Get latest enemy position from pathfinder in mm, or None if unavailable."""
        if self.pathfinder.enemy_x is None or self.pathfinder.enemy_y is None:
            return None
        return (self.pathfinder.enemy_x * 1000.0, self.pathfinder.enemy_y * 1000.0)

    @staticmethod
    def _distance_mm(x1: float, y1: float, x2: float, y2: float) -> float:
        """Euclidean distance in mm."""
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def _get_batch_enemy_distances_mm(self, batch: Dict, pickup_pose: Dict,
                                      enemy_x_mm: float, enemy_y_mm: float) -> Tuple[float, float, float]:
        """
        Returns distances from enemy to:
          (batch_center_mm, pickup_pose_mm, min_of_both_mm)
        """
        # batch center is in meters in active batch info
        center_x_mm = batch['center'].x * 1000.0
        center_y_mm = batch['center'].y * 1000.0

        pickup_x_mm = float(pickup_pose['x'])
        pickup_y_mm = float(pickup_pose['y'])

        d_center = self._distance_mm(enemy_x_mm, enemy_y_mm, center_x_mm, center_y_mm)
        d_pickup = self._distance_mm(enemy_x_mm, enemy_y_mm, pickup_x_mm, pickup_y_mm)
        return d_center, d_pickup, min(d_center, d_pickup)

    def _is_hard_blocked_by_enemy(self, batch: Dict, pickup_pose: Dict,
                                  enemy_x_mm: float, enemy_y_mm: float) -> bool:
        """Return True if enemy is too close to batch center or pickup pose."""
        _, _, min_distance = self._get_batch_enemy_distances_mm(
            batch, pickup_pose, enemy_x_mm, enemy_y_mm
        )
        return min_distance <= self.hard_block_radius_mm

    def _compute_enemy_penalty(self, batch: Dict, pickup_pose: Dict,
                               enemy_x_mm: float, enemy_y_mm: float) -> float:
        """
        Compute a score penalty based on enemy proximity.
        0 outside near_penalty_radius_mm, increasing as enemy gets closer.
        """
        _, _, min_distance = self._get_batch_enemy_distances_mm(
            batch, pickup_pose, enemy_x_mm, enemy_y_mm
        )

        if min_distance >= self.near_penalty_radius_mm:
            return 0.0

        # Guard against invalid configuration
        effective_span = max(1.0, self.near_penalty_radius_mm - self.hard_block_radius_mm)

        # proximity in [0,1], where 1 is right at hard block radius
        proximity = (self.near_penalty_radius_mm - max(min_distance, self.hard_block_radius_mm)) / effective_span

        max_penalty = 2.0
        return proximity * max_penalty
    
    def set_front_status(self, is_full: bool):
        """Set front picker status"""
        self.front_full = is_full
        status = "FULL" if is_full else "EMPTY"
        self.strategy.get_logger().info(f"🔧 Front picker: {status}")
    
    def set_back_status(self, is_full: bool):
        """Set back picker status"""
        self.back_full = is_full
        status = "FULL" if is_full else "EMPTY"
        self.strategy.get_logger().info(f"🔧 Back picker: {status}")
    
    def is_front_available(self) -> bool:
        """Check if front picker is available (empty)"""
        return not self.front_full
    
    def is_back_available(self) -> bool:
        """Check if back picker is available (empty)"""
        return not self.back_full
    
    def get_available_device(self) -> Optional[str]:
        """
        Get an available device (front or back) for pickup.
        Simple fallback when no angle optimization is possible.
        
        Returns:
            'ESP_front' or 'ESP_back' if available, None if both full
        """
        if self.is_front_available():
            return 'ESP_front'
        elif self.is_back_available():
            return 'ESP_back'
        else:
            self.strategy.get_logger().warning("⚠️ Both front and back pickers are full!")
            return None
    
    def get_optimal_device_from_path(self, stm32_commands: List[str]) -> Optional[str]:
        """
        Select the optimal device (front or back) based on actual path trajectory.
        Analyzes the initial direction of movement to minimize rotations.
        
        Args:
            stm32_commands: List of path waypoint commands from pathfinder
        
        Returns:
            'ESP_front', 'ESP_back', or None if both full
        """
        # Get current robot pose
        robot_pose = self.strategy.latest_robot_pose
        if robot_pose is None:
            self.strategy.get_logger().warning('⚠️ Robot pose unavailable - using simple device selection')
            return self.get_available_device()
        
        if not stm32_commands:
            self.strategy.get_logger().warning('⚠️ No path waypoints - using simple device selection')
            return self.get_available_device()
        
        # Parse first waypoint to determine initial movement direction
        first_cmd = stm32_commands[0]
        values = first_cmd.replace('go_to(', '').replace(')', '').split(',')
        first_waypoint_x = float(values[0].strip())
        first_waypoint_y = float(values[1].strip())
        
        # Calculate direction vector from robot to first waypoint
        dx = first_waypoint_x - robot_pose.x * 1000  # Convert to mm
        dy = first_waypoint_y - robot_pose.y * 1000
        
        # Calculate angle to first waypoint
        target_direction = math.degrees(math.atan2(dy, dx))
        
        # Get robot's current orientation
        robot_theta_deg = math.degrees(robot_pose.z) if robot_pose.z < 6.28 else robot_pose.z
        
        # Calculate how much the robot needs to rotate
        angle_to_waypoint = circular_angle_difference(robot_theta_deg, target_direction)
        
        # Determine which device requires less rotation:
        # - Front picker: efficient when robot is facing toward path (angle < 90°)
        # - Back picker: efficient when robot is facing away from path (angle > 90°)
        prefer_back = angle_to_waypoint > 90
        
        # Check availability with preference
        front_available = self.is_front_available()
        back_available = self.is_back_available()
        
        if not front_available and not back_available:
            self.strategy.get_logger().warning("⚠️ Both front and back pickers are full!")
            return None
        
        # Apply preference based on path direction
        if prefer_back:
            if back_available:
                self.strategy.get_logger().info(
                    f'🎯 Back picker selected (path angle={angle_to_waypoint:.1f}° > 90°, '
                    f'robot={robot_theta_deg:.1f}°, path_dir={target_direction:.1f}°) - backwards motion optimal'
                )
                return 'ESP_back'
            elif front_available:
                self.strategy.get_logger().info(
                    f'🔄 Front picker selected (back preferred but unavailable, path angle={angle_to_waypoint:.1f}°)'
                )
                return 'ESP_front'
        else:
            if front_available:
                self.strategy.get_logger().info(
                    f'🎯 Front picker selected (path angle={angle_to_waypoint:.1f}° ≤ 90°, '
                    f'robot={robot_theta_deg:.1f}°, path_dir={target_direction:.1f}°) - forward motion optimal'
                )
                return 'ESP_front'
            elif back_available:
                self.strategy.get_logger().info(
                    f'🔄 Back picker selected (front preferred but unavailable, path angle={angle_to_waypoint:.1f}°)'
                )
                return 'ESP_back'
        
        return None
    
    def update_available_batches(self, force_refresh: bool = False):
        """Update list of available batches from pathfinder/map"""
        try:
            meta = self.batch_registry.metadata()
            registry_version = int(meta.get('version', -1))
            registry_stale = bool(meta.get('stale', False))

            # Fast path: no new registry version and local cache already populated.
            if (
                not force_refresh
                and self.available_batches
                and not registry_stale
                and registry_version == self._last_registry_version
            ):
                return

            snapshot = self.batch_registry.get_snapshot()

            # Fallback once if snapshot is empty (e.g. very first cycle before background thread updates)
            if not snapshot:
                batch_info = self.pathfinder.get_active_batches()
                if batch_info is not None:
                    self.batch_registry.update_from_batch_info(batch_info)
                    snapshot = self.batch_registry.get_snapshot()
                    meta = self.batch_registry.metadata()
                    registry_version = int(meta.get('version', -1))
                    registry_stale = bool(meta.get('stale', False))

            if not snapshot:
                self.strategy.get_logger().warning('⚠️ Failed to get active batches from registry/pathfinder')
                return

            self.available_batches = snapshot
            self.last_batch_update = time.time()
            self._last_registry_version = registry_version

            # Throttle logs to avoid flooding in hot decision loop
            now = time.time()
            if now - self._last_batch_sync_log_s < 1.0 and not force_refresh:
                return

            fx_count = sum(1 for b in self.available_batches.values() if b['batch_type'] == 'fx')
            sx_count = sum(1 for b in self.available_batches.values() if b['batch_type'] == 'sx')
            stale_suffix = f', stale={registry_stale}'
            if meta.get('last_error'):
                stale_suffix += f', last_error={meta["last_error"]}'

            self.strategy.get_logger().info(
                f'🧠 Batch list updated: {len(self.available_batches)} batches '
                f'(FX={fx_count}, SX={sx_count}, version={registry_version}{stale_suffix})'
            )
            self._log_available_batches()
            self._last_batch_sync_log_s = now
        
        except Exception as e:
            self.strategy.get_logger().error(f'❌ Error updating batch list: {e}')

    def _log_available_batches(self):
        """Log the currently available batches in a compact, readable format."""
        if not self.available_batches:
            self.strategy.get_logger().info('📦 Available batches: none')
            return

        ordered = sorted(self.available_batches.values(), key=lambda b: str(b.get('known_id', '')))
        parts = []
        for batch in ordered:
            known_id = batch.get('known_id', '?')
            batch_type = batch.get('batch_type', '?')
            members = batch.get('member_count', '?')
            quality = float(batch.get('quality', 0.0))
            parts.append(f'{known_id}[{batch_type}] m={members} q={quality:.0f}')

        self.strategy.get_logger().info(f'📦 Available batches: {", ".join(parts)}')
    
    def get_pickup_poses(self, known_id: str) -> Optional[List[Dict]]:
        """
        Get pickup poses for a specific batch by known_id
        
        Args:
            known_id: Batch known_id (e.g., 'S1', 'S2', 'S1_WEB', etc.)
        
        Returns:
            List of pickup pose dictionaries or None if not found
        """
        # Strip _WEB suffix for simulation batches (S1_WEB → S1)
        base_id = known_id.upper().replace('_WEB', '')
        return PICKUP_POSES.get(base_id)
    
    def find_optimal_putdown_position(self, center_x: float, center_y: float, 
                                      reference_x: float = None, reference_y: float = None) -> Optional[Dict]:
        """
        Find optimal putdown position within 30cm radius circle of a type B zone.
        The putdown point is positioned to approach the center point.
        
        Args:
            center_x: Center x coordinate in mm
            center_y: Center y coordinate in mm
            reference_x: Reference x position (e.g., pickup position) in mm, or None to use robot pose
            reference_y: Reference y position (e.g., pickup position) in mm, or None to use robot pose
        
        Returns:
            Dictionary with 'x', 'y', 'theta' for putdown position, or None if no reference available
        """
        # Use provided reference position, otherwise fall back to robot pose
        if reference_x is not None and reference_y is not None:
            robot_x_mm = reference_x
            robot_y_mm = reference_y
            self.strategy.get_logger().info(
                f'📍 Using reference position: ({robot_x_mm:.0f}mm, {robot_y_mm:.0f}mm)'
            )
        else:
            robot_pose = self.strategy.latest_robot_pose
            if robot_pose is None:
                self.strategy.get_logger().warning('⚠️ Robot pose not available for putdown optimization')
                return None
            robot_x_mm = robot_pose.x * 1000
            robot_y_mm = robot_pose.y * 1000
        
        # Calculate vector from center to robot
        dx = robot_x_mm - center_x
        dy = robot_y_mm - center_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # If robot is very close to center, use a default position
        if distance < 50:  # Less than 5cm
            self.strategy.get_logger().warning('⚠️ Robot too close to center, using default offset')
            dx = 300
            dy = 0
            distance = 300
        
        # Normalize direction vector
        dx_norm = dx / distance
        dy_norm = dy / distance
        
        # Place putdown point 30cm (300mm) from center toward robot's direction
        putdown_x = center_x + dx_norm * 300
        putdown_y = center_y + dy_norm * 300
        
        # Calculate angle pointing toward center
        angle_to_center = math.degrees(math.atan2(-dy_norm, -dx_norm))
        
        self.strategy.get_logger().info(
            f'📍 Optimal putdown: ({putdown_x:.0f}mm, {putdown_y:.0f}mm, {angle_to_center:.1f}°) '
            f'from center ({center_x:.0f}mm, {center_y:.0f}mm)'
        )
        
        return {
            'x': putdown_x,
            'y': putdown_y,
            'theta': angle_to_center
        }
    
    def choose_best_pickup_pose(self, known_id: str) -> Optional[Dict]:
        """
        Choose the best pickup pose for a batch based on robot position
        
        Args:
            known_id: Batch known_id
        
        Returns:
            Best pickup pose dict with 'x', 'y', 'theta' keys, or None if not found
        """
        poses = self.get_pickup_poses(known_id)
        if not poses:
            self.strategy.get_logger().warning(f'⚠️ No pickup poses found for {known_id}')
            return None
        
        # If only one pose, return it
        if len(poses) == 1:
            return poses[0]
        
        # Get current robot position
        robot_pose = self.strategy.latest_robot_pose
        if robot_pose is None:
            self.strategy.get_logger().warning('⚠️ Robot pose not available, using first pose')
            return poses[0]
        
        # Calculate distance to each pose and return closest
        robot_x_mm = robot_pose.x * 1000
        robot_y_mm = robot_pose.y * 1000
        
        def distance(pose):
            dx = pose['x'] - robot_x_mm
            dy = pose['y'] - robot_y_mm
            return math.sqrt(dx*dx + dy*dy)
        
        best_pose = min(poses, key=distance)
        self.strategy.get_logger().info(
            f'📍 Chose pickup pose for {known_id}: x={best_pose["x"]}, y={best_pose["y"]} '
            f'(distance: {distance(best_pose):.0f}mm)'
        )
        return best_pose

    @staticmethod
    def _choose_best_pickup_pose_fast(poses: List[Dict], robot_x_mm: float, robot_y_mm: float) -> Optional[Dict]:
        """Fast pickup pose selection without extra ROS reads/logs in hot loop."""
        if not poses:
            return None
        if len(poses) == 1:
            return poses[0]

        best_pose = None
        best_dist_sq = float('inf')
        for pose in poses:
            dx = float(pose['x']) - robot_x_mm
            dy = float(pose['y']) - robot_y_mm
            d2 = dx * dx + dy * dy
            if d2 < best_dist_sq:
                best_dist_sq = d2
                best_pose = pose
        return best_pose
    
    def choose_best_putdown_zone(self, reference_x: float = None, reference_y: float = None) -> Optional[Tuple[str, Dict]]:
        """
        Choose the best putdown zone from available type B zones based on proximity.
        Type A zones are skipped for now.
        
        Args:
            reference_x: Reference x position (e.g., pickup position) in mm, or None to use robot pose
            reference_y: Reference y position (e.g., pickup position) in mm, or None to use robot pose
        
        Returns:
            Tuple of (zone_name, zone_dict) with zone information, or None if no reference available
        """
        zone_timer = time.perf_counter()

        # Use provided reference position, otherwise fall back to robot pose
        if reference_x is not None and reference_y is not None:
            robot_x_mm = reference_x
            robot_y_mm = reference_y
            self.strategy.get_logger().info(
                f'🎯 Selecting putdown zone from reference: ({robot_x_mm:.0f}mm, {robot_y_mm:.0f}mm)'
            )
        else:
            robot_pose = self.strategy.latest_robot_pose
            if robot_pose is None:
                self.strategy.get_logger().warning('⚠️ Robot pose not available for putdown zone selection')
                return None
            robot_x_mm = robot_pose.x * 1000
            robot_y_mm = robot_pose.y * 1000
        
        # Filter for type B zones only
        type_b_zones = {name: zone for name, zone in PUTDOWN_POSES.items() if zone['type'] == 'B'}
        
        if not type_b_zones:
            self.strategy.get_logger().warning('⚠️ No type B putdown zones available')
            return None
        
        # Find closest type B zone that is free of fx obstacles
        # Zone names are 'fxN' -> corresponding fx batch id is 'fN'
        best_zone = None
        best_distance = float('inf')
        best_name = None
        
        # Get team color once (e.g. 'yellow' -> 'y', 'blue' -> 'b')
        team_color_char = 'y' if getattr(self.strategy, 'color', None) == 'yellow' else 'b'

        for zone_name, zone in type_b_zones.items():
            # Derive fx batch id from zone name: 'fx3' -> 'f3'
            fx_batch_id = zone_name.replace('fx', 'f', 1)
            batch = self.available_batches.get(fx_batch_id)
            if batch is not None and batch.get('member_count', 0) > 0:
                # Query the color pattern to see if majority are our color
                from geometry_msgs.msg import Point as _Point
                color_pattern, cp_success, _, _, _ = self.pathfinder.get_batch_color(
                    fx_batch_id, _Point()
                )
                if cp_success and color_pattern:
                    our_count = color_pattern.lower().count(team_color_char)
                    total = len(color_pattern)
                    majority_ours = our_count > total / 2
                else:
                    # Could not get pattern — be conservative and block the zone
                    majority_ours = True

                if majority_ours:
                    self.strategy.get_logger().info(
                        f'⚠️ Putdown zone {zone_name} blocked: batch {fx_batch_id} '
                        f'has {batch["member_count"]} members, majority our color '
                        f'("{color_pattern}" our_char="{team_color_char}"), skipping'
                    )
                    continue
                else:
                    self.strategy.get_logger().info(
                        f'✅ Putdown zone {zone_name}: batch {fx_batch_id} has members '
                        f'but majority are NOT our color ("{color_pattern}" our_char="{team_color_char}"), zone usable'
                    )

            dx = zone['x'] - robot_x_mm
            dy = zone['y'] - robot_y_mm
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < best_distance:
                best_distance = distance
                best_zone = zone
                best_name = zone_name
        
        if best_zone is None:
            self.strategy.get_logger().warning('⚠️ All type B putdown zones are blocked by obstacles')
            return None

        self.strategy.get_logger().info(
            f'🎯 Selected putdown zone: {best_name} at ({best_zone["x"]}mm, {best_zone["y"]}mm) '
            f'- distance: {best_distance:.0f}mm'
        )
        self._log_duration_ms('choose_best_putdown_zone total', zone_timer)
        
        return (best_name, best_zone)
    
    def get_optimal_batch(self, batch_type: Optional[str] = None, 
                          min_members: int = 1) -> Optional[Dict]:
        """
        Find the optimal batch to steal based on robot position and batch properties
        
        Args:
            batch_type: Filter by batch type ('fx', 'sx'), or None for all
            min_members: Minimum number of members required
        
        Returns:
            Dictionary with batch info, pickup_pose, distance, and score, or None if no batches
        """
        total_timer = time.perf_counter()

        # Update batch list
        update_timer = time.perf_counter()
        self.update_available_batches()
        self._log_duration_ms('get_optimal_batch update_available_batches', update_timer)
        
        # Get current robot position
        robot_pose = self.strategy.latest_robot_pose
        if robot_pose is None:
            self.strategy.get_logger().warning('⚠️ Cannot determine optimal batch: robot pose not available')
            return None
        
        robot_x_mm = robot_pose.x * 1000
        robot_y_mm = robot_pose.y * 1000

        enemy_pos_mm = self._get_enemy_position_mm()
        if enemy_pos_mm is not None:
            self.strategy.get_logger().info(
                f'⚠️ Enemy considered in scoring: hard_block_radius={self.hard_block_radius_mm:.0f}mm, '
                f'near_penalty_radius={self.near_penalty_radius_mm:.0f}mm'
            )
        
        # Filter batches
        batches = list(self.available_batches.values())
        
        if batch_type:
            batches = [b for b in batches if b['batch_type'] == batch_type]
        
        batches = [b for b in batches if b['member_count'] >= min_members]
        
        # Filter batches that have pickup poses (mainly SX batches)
        batches_with_poses = []
        for batch in batches:
            known_id = batch['known_id']

            # Prefer pre-resolved pickup poses from registry snapshot (fast path)
            pickup_pose = None
            poses = batch.get('pickup_poses')
            if poses:
                pickup_pose = self._choose_best_pickup_pose_fast(poses, robot_x_mm, robot_y_mm)
            else:
                # Fallback for compatibility with legacy/non-registry data
                pickup_pose = self.choose_best_pickup_pose(known_id)

            if pickup_pose:
                batches_with_poses.append({
                    'batch': batch,
                    'pickup_pose': pickup_pose
                })
        
        if not batches_with_poses:
            self.strategy.get_logger().info('ℹ️ No batches available with pickup poses')
            return None
        
        # Score each batch
        scored_batches = []
        
        for item in batches_with_poses:
            batch = item['batch']
            pickup_pose = item['pickup_pose']

            # Hard reject if enemy blocks this batch/pickup area
            if enemy_pos_mm is not None:
                enemy_x_mm, enemy_y_mm = enemy_pos_mm
                if self._is_hard_blocked_by_enemy(batch, pickup_pose, enemy_x_mm, enemy_y_mm):
                    _, _, min_enemy_dist = self._get_batch_enemy_distances_mm(
                        batch, pickup_pose, enemy_x_mm, enemy_y_mm
                    )
                    self.strategy.get_logger().info(
                        f'🚫 Skipping batch {batch["known_id"]}: enemy too close '
                        f'({min_enemy_dist:.0f}mm <= {self.hard_block_radius_mm:.0f}mm)'
                    )
                    continue
            
            # Calculate distance to pickup point
            dx = pickup_pose['x'] - robot_x_mm
            dy = pickup_pose['y'] - robot_y_mm
            distance_mm = math.sqrt(dx*dx + dy*dy)
            
            # Calculate score (closer is better, more members is better)
            # Normalize factors
            max_distance = 3000  # mm
            distance_score = max(0, 1 - (distance_mm / max_distance))
            
            max_members = 6
            member_score = min(1.0, batch['member_count'] / max_members)
            
            quality_score = batch['quality'] / 100.0
            
            # Weighted total score
            total_score = (
                distance_score * 1.0 +
                member_score * 2.0 +
                quality_score * 1.5
            )

            enemy_penalty = 0.0
            if enemy_pos_mm is not None:
                enemy_x_mm, enemy_y_mm = enemy_pos_mm
                enemy_penalty = self._compute_enemy_penalty(batch, pickup_pose, enemy_x_mm, enemy_y_mm)
                total_score -= enemy_penalty
            
            scored_batches.append({
                'known_id': batch['known_id'],
                'batch_type': batch['batch_type'],
                'member_count': batch['member_count'],
                'direction': batch['direction'],
                'center': batch['center'],
                'quality': batch['quality'],
                'pickup_pose': pickup_pose,
                'distance_mm': distance_mm,
                'score': total_score,
                'enemy_penalty': enemy_penalty
            })

        if not scored_batches:
            self.strategy.get_logger().info('ℹ️ No valid batches left after enemy blocking filter')
            return None
        
        # Sort by score (highest first)
        scored_batches.sort(key=lambda x: x['score'], reverse=True)
        
        best = scored_batches[0]
        self.strategy.get_logger().info(
            f'🎯 Optimal batch: {best["known_id"]} (type={best["batch_type"]}, '
            f'members={best["member_count"]}, distance={best["distance_mm"]:.0f}mm, '
            f'enemy_penalty={best.get("enemy_penalty", 0.0):.2f}, score={best["score"]:.2f})'
        )
        self._log_duration_ms('get_optimal_batch total', total_timer)
        
        return best
    
    def steal_batch(self, batch_type: Optional[str] = None, 
                    device: Optional[str] = None,
                    min_members: int = 1) -> Optional[List[Dict]]:
        """
        Generate complete tasks to steal a batch: pickup task + putdown task
        
        Args:
            batch_type: Type of batch to steal ('fx', 'sx'), or None for any
            device: Device to use ('ESP_front' or 'ESP_back'), or None to auto-select
            min_members: Minimum number of members required
        
        Returns:
            List containing [pickup_task, putdown_task], or None if generation fails
        """
        steal_timer = time.perf_counter()

        self.strategy.get_logger().info(
            f'🎯 steal_batch() called - searching for optimal batch '
            f'(type={batch_type or "any"}, min_members={min_members})'
        )
        
        # Get optimal batch
        phase_timer = time.perf_counter()
        optimal_batch = self.get_optimal_batch(batch_type, min_members)
        self._log_duration_ms('steal_batch phase select_optimal_batch', phase_timer)
        
        if optimal_batch is None:
            self.strategy.get_logger().warning('⚠️ No batches available to steal')
            self._log_duration_ms('steal_batch total (failed: no optimal batch)', steal_timer)
            return None
        
        # Extract batch information
        known_id = optimal_batch['known_id']
        pickup_pose = optimal_batch['pickup_pose']
        
        # Convert pickup pose coordinates
        pickup_x_mm = pickup_pose['x']
        pickup_y_mm = pickup_pose['y']
        pickup_angles = pickup_pose['theta']
        
        # Choose angle closest to current robot orientation
        robot_pose = self.strategy.latest_robot_pose
        if robot_pose and len(pickup_angles) > 1:
            robot_theta_deg = math.degrees(robot_pose.z) if robot_pose.z < 6.28 else robot_pose.z
            # Use circular difference to handle wrap-around (e.g., -179° vs 179°)
            angle_diffs = [circular_angle_difference(robot_theta_deg, angle) for angle in pickup_angles]
            pickup_theta = pickup_angles[angle_diffs.index(min(angle_diffs))]
        else:
            pickup_theta = pickup_angles[0]
        
        self.strategy.get_logger().info(
            f'🎯 Selected batch {known_id}: '
            f'pos=({pickup_x_mm}mm, {pickup_y_mm}mm), '
            f'angle={pickup_theta}°, '
            f'members={optimal_batch["member_count"]}, '
            f'distance={optimal_batch["distance_mm"]:.0f}mm'
        )
        
        # Generate path using pathfinder BEFORE selecting device
        self.strategy.get_logger().info('🗺️  Generating path to batch...')
        
        # Get batch direction and center for proper approach positioning
        batch_direction = optimal_batch['direction']
        batch_center = optimal_batch['center']
        
        # Adjust goal position based on batch orientation (same logic as strategy_manager.handle_global_goal)
        # This ensures proper approach for horizontal/vertical batches
        pickup_x = pickup_x_mm / 1000.0  # Convert to meters
        pickup_y = pickup_y_mm / 1000.0
        
        if batch_direction == "horizontal" and pickup_y < batch_center.y:
            # Approaching from below a horizontal batch
            adjusted_y = pickup_y - 0.1
            adjusted_x = pickup_x
            self.strategy.get_logger().info(f'📐 Horizontal batch - approaching from below: y offset = -0.1m')
        elif batch_direction == "horizontal" and pickup_y > batch_center.y:
            # Approaching from above a horizontal batch
            adjusted_y = pickup_y + 0.1
            adjusted_x = pickup_x
            self.strategy.get_logger().info(f'📐 Horizontal batch - approaching from above: y offset = +0.1m')
        elif batch_direction == "vertical" and pickup_x < batch_center.x:
            # Approaching from left of a vertical batch
            adjusted_x = pickup_x - 0.1
            adjusted_y = pickup_y
            self.strategy.get_logger().info(f'📐 Vertical batch - approaching from left: x offset = -0.1m')
        elif batch_direction == "vertical" and pickup_x > batch_center.x:
            # Approaching from right of a vertical batch
            adjusted_x = pickup_x + 0.1
            adjusted_y = pickup_y
            self.strategy.get_logger().info(f'📐 Vertical batch - approaching from right: x offset = +0.1m')
        else:
            # Default case: no adjustment needed
            adjusted_x = pickup_x
            adjusted_y = pickup_y
            self.strategy.get_logger().warning(
                f'⚠️ Edge case: position equals batch center or unexpected direction "{batch_direction}"'
            )
        
        # Set goal for pathfinder with adjusted position
        # Convert pickup_theta from degrees to radians (pathfinder expects radians)
        pickup_theta_rad = math.radians(pickup_theta)
        self.pathfinder.set_goal(adjusted_x, adjusted_y, pickup_theta_rad)
        
        # Get path waypoints from pathfinder
        phase_timer = time.perf_counter()
        stm32_commands, path_success = self.pathfinder.path()
        pickup_waypoint_count = len(stm32_commands) if stm32_commands else 0
        self.strategy.get_logger().info(
            f'⏱️ steal_batch phase pickup_path: {(time.perf_counter() - phase_timer) * 1000.0:.1f}ms '
            f'({pickup_waypoint_count} waypoint commands)'
        )
        
        if not path_success:
            self.strategy.get_logger().error(f'❌ Failed to generate path to {known_id}')
            self._log_duration_ms('steal_batch total (failed: pickup path)', steal_timer)
            return None
        
        self.strategy.get_logger().info(f'✅ Path generated: {len(stm32_commands)} waypoints')
        
        # NOW select device based on actual path trajectory (if not manually specified)
        if device is None:
            phase_timer = time.perf_counter()
            device = self.get_optimal_device_from_path(stm32_commands)
            self._log_duration_ms('steal_batch phase select_device_from_path', phase_timer)
            if device is None:
                self.strategy.get_logger().error('❌ No available device (both front and back are full)')
                self._log_duration_ms('steal_batch total (failed: no device)', steal_timer)
                return None
            self.strategy.get_logger().info(f'🔧 Selected device: {device}')
        
        # Build task sequence with STM32 waypoints + ESP command (like static YAML tasks)
        task_sequence = []
        
        # Add STM32 waypoint commands
        last_cmd = None
        for cmd in stm32_commands:
            # Parse waypoint: "go_to(x, y, speed)"
            values = cmd.replace('go_to(', '').replace(')', '').split(',')
            x = float(values[0].strip())
            y = float(values[1].strip())
            speed = int(values[2].strip())
            
            # Avoid duplicate consecutive waypoints
            if cmd != last_cmd:
                # Choose command based on device (front=forward, back=backward)
                if device == 'ESP_back':
                    stm32_cmd = 'go_to_backwards'
                else:
                    stm32_cmd = 'go_to'
                
                task_sequence.append({
                    'device': 'STM',
                    'command': stm32_cmd,
                    'params': [x, y, 1100.0, {'from_pathfinding': True}],
                    'wait': False
                })
            last_cmd = cmd
        
        # Add final orientation command if goal angle is available
        goal_angle = self.pathfinder.goal_angle
        if goal_angle is not None:
            # Add final movement to pickup position (like strategy_manager path() logic)
            # This ensures proper alignment after orientation
            if device == 'ESP_front':
                task_sequence.append({
                    'device': 'STM',
                    'command': 'go_to',
                    'params': [pickup_x_mm, pickup_y_mm, 1100.0, 1.0, {'from_pathfinding': True}],
                    'wait': False
                })
                self.strategy.get_logger().info(
                    f'📍 Added final go_to after orientate: x={pickup_x_mm:.0f}mm, y={pickup_y_mm:.0f}mm'
                )
            elif device == 'ESP_back':
                task_sequence.append({
                    'device': 'STM',
                    'command': 'go_to_backwards',
                    'params': [pickup_x_mm, pickup_y_mm, 1100.0, 1.0, {'from_pathfinding': True}],
                    'wait': False
                })
                self.strategy.get_logger().info(
                    f'📍 Added final go_to_backwards after orientate: x={pickup_x_mm:.0f}mm, y={pickup_y_mm:.0f}mm'
                )
        
        # Add ESP pick_up command at the end
        task_sequence.append({
            'device': device,
            'command': 'pick_up',
            'params': {},
            'wait': True  # Wait for STM32 to reach position before picking
        })
        
        self.strategy.get_logger().info(
            f'📋 Task sequence for {device}: {len([c for c in task_sequence if c["device"] == "STM"])} STM32 commands + '
            f'1 {device} pickup command'
        )
        
        # Create pickup task structure (compatible with load_task_to_queues)
        pickup_task = {
            'name': f'steal_{known_id}_{device.split("_")[1]}',
            'sequence': task_sequence,
            'global_goal': {
                'x': pickup_x_mm,
                'y': pickup_y_mm,
                'theta': pickup_theta
            }
        }
        
        # NOTE: Picker status should be updated by the executor after successful pickup,
        # not here during task generation. Premature status updates can cause stale state
        # if the task fails or is interrupted.
        
        self.strategy.get_logger().info(f'✅ Generated pickup task: {pickup_task["name"]}')

        # Exclude picked batch from obstacle set for subsequent path generation (e.g., putdown path)
        # while keeping its metadata queryable by known_id.
        if hasattr(self.pathfinder, 'add_untracked_batch'):
            self.pathfinder.add_untracked_batch(known_id)
        
        # ========== NOW CREATE PUTDOWN TASK ==========
        
        # Select best putdown zone (type B only for now) based on pickup position
        phase_timer = time.perf_counter()
        putdown_zone_result = self.choose_best_putdown_zone(
            reference_x=pickup_x_mm,
            reference_y=pickup_y_mm
        )
        self._log_duration_ms('steal_batch phase choose_putdown_zone', phase_timer)
        if putdown_zone_result is None:
            self.strategy.get_logger().warning('⚠️ No putdown zone available, returning pickup task only')
            if hasattr(self.pathfinder, 'remove_untracked_batch'):
                self.pathfinder.remove_untracked_batch(known_id)
            self._log_duration_ms('steal_batch total (pickup only)', steal_timer)
            return [pickup_task]
        
        zone_name, zone = putdown_zone_result

        # If the selected zone has an fx batch with enemy-majority members,
        # untrack it so the pathfinder ignores it as an obstacle during putdown path planning.
        putdown_zone_fx_batch_id = zone_name.replace('fx', 'f', 1)
        putdown_zone_fx_untracked = False
        putdown_zone_batch = self.available_batches.get(putdown_zone_fx_batch_id)
        if putdown_zone_batch is not None and putdown_zone_batch.get('member_count', 0) > 0:
            from geometry_msgs.msg import Point as _Point
            team_color_char = 'y' if getattr(self.strategy, 'color', None) == 'yellow' else 'b'
            color_pattern, cp_success, _, _, _ = self.pathfinder.get_batch_color(
                putdown_zone_fx_batch_id, _Point()
            )
            if cp_success and color_pattern:
                our_count = color_pattern.lower().count(team_color_char)
                majority_ours = our_count > len(color_pattern) / 2
            else:
                majority_ours = False  # unknown -> treat as enemy, untrack it
            if not majority_ours and hasattr(self.pathfinder, 'add_untracked_batch'):
                self.pathfinder.add_untracked_batch(putdown_zone_fx_batch_id)
                putdown_zone_fx_untracked = True
                self.strategy.get_logger().info(
                    f'🙈 Untracking enemy batch {putdown_zone_fx_batch_id} at putdown zone {zone_name} '
                    f'for path planning (pattern="{color_pattern}")'
                )

        # Calculate optimal putdown position within 30cm radius circle
        # Use pickup position as reference (robot will be there after pickup)
        phase_timer = time.perf_counter()
        putdown_pos = self.find_optimal_putdown_position(
            zone['x'], 
            zone['y'],
            reference_x=pickup_x_mm,
            reference_y=pickup_y_mm
        )
        self._log_duration_ms('steal_batch phase compute_putdown_position', phase_timer)
        if putdown_pos is None:
            self.strategy.get_logger().warning('⚠️ Could not calculate putdown position, returning pickup task only')
            if hasattr(self.pathfinder, 'remove_untracked_batch'):
                self.pathfinder.remove_untracked_batch(known_id)
            if putdown_zone_fx_untracked and hasattr(self.pathfinder, 'remove_untracked_batch'):
                self.pathfinder.remove_untracked_batch(putdown_zone_fx_batch_id)
            self._log_duration_ms('steal_batch total (pickup only: no putdown pos)', steal_timer)
            return [pickup_task]
        
        # Create putdown task using the same device
        phase_timer = time.perf_counter()
        putdown_task = self.create_put_down_task(
            device=device,
            target_x=putdown_pos['x'],
            target_y=putdown_pos['y'],
            target_theta=putdown_pos['theta'],
            center_x=zone['x'],
            center_y=zone['y'],
            start_x=pickup_x_mm,
            start_y=pickup_y_mm,
            start_theta=pickup_theta
        )
        self._log_duration_ms('steal_batch phase create_put_down_task', phase_timer)
        
        if putdown_task is None:
            self.strategy.get_logger().warning('⚠️ Failed to generate putdown task, returning pickup task only')
            if hasattr(self.pathfinder, 'remove_untracked_batch'):
                self.pathfinder.remove_untracked_batch(known_id)
            if putdown_zone_fx_untracked and hasattr(self.pathfinder, 'remove_untracked_batch'):
                self.pathfinder.remove_untracked_batch(putdown_zone_fx_batch_id)
            self._log_duration_ms('steal_batch total (pickup only: putdown failed)', steal_timer)
            return [pickup_task]

        # Putdown task is now generated. Restore default obstacle tracking for future planning calls.
        if hasattr(self.pathfinder, 'remove_untracked_batch'):
            self.pathfinder.remove_untracked_batch(known_id)
        if putdown_zone_fx_untracked and hasattr(self.pathfinder, 'remove_untracked_batch'):
            self.pathfinder.remove_untracked_batch(putdown_zone_fx_batch_id)
            self.strategy.get_logger().info(
                f'👁️ Restored tracking of batch {putdown_zone_fx_batch_id} after putdown path planning'
            )
        
        self.strategy.get_logger().info(f'✅ Generated complete steal sequence: pickup + putdown at {zone_name}')
        self._log_duration_ms('steal_batch total', steal_timer)
        return [pickup_task, putdown_task]
    
    def create_put_down_task(self, device: str, target_x: float, target_y: float,
                            target_theta: float, center_x: Optional[float] = None,
                            center_y: Optional[float] = None, start_x: Optional[float] = None,
                            start_y: Optional[float] = None, start_theta: Optional[float] = None) -> Optional[Dict]:
        """
        Create a put_down task to drop batch at target location
        
        Args:
            device: Device that has the batch ('ESP_front' or 'ESP_back')
            target_x: Target x coordinate in mm
            target_y: Target y coordinate in mm
            target_theta: Target orientation in degrees
            center_x: Optional putdown zone center x in mm (for diameter clearing sweep)
            center_y: Optional putdown zone center y in mm (for diameter clearing sweep)
            start_x: Optional start x in mm (if provided, overrides live robot pose)
            start_y: Optional start y in mm (if provided, overrides live robot pose)
            start_theta: Optional start theta in degrees (if provided, overrides live robot pose)
        
        Returns:
            Task dictionary for putting down batch, or None if path generation fails
        """
        total_timer = time.perf_counter()

        self.strategy.get_logger().info(
            f'📦 Creating put_down task: device={device}, '
            f'target=({target_x}mm, {target_y}mm, {target_theta}°)'
        )
        
        # Convert theta from degrees to radians for pathfinder
        target_theta_rad = math.radians(target_theta)
        
        # Generate path using pathfinder
        self.pathfinder.set_goal(
            target_x / 1000.0,
            target_y / 1000.0,
            target_theta_rad
        )
        
        # Get path waypoints (use explicit start pose if provided)
        path_timer = time.perf_counter()
        if start_x is not None and start_y is not None and start_theta is not None:
            start_theta_rad = math.radians(start_theta)
            stm32_commands, path_success = self.pathfinder.path(
                start_x=start_x / 1000.0,
                start_y=start_y / 1000.0,
                start_theta=start_theta_rad
            )
        else:
            stm32_commands, path_success = self.pathfinder.path()
        putdown_waypoint_count = len(stm32_commands) if stm32_commands else 0
        self.strategy.get_logger().info(
            f'⏱️ create_put_down_task path_generation: {(time.perf_counter() - path_timer) * 1000.0:.1f}ms '
            f'({putdown_waypoint_count} waypoint commands)'
        )
        
        if not path_success:
            self.strategy.get_logger().error('❌ Failed to generate path for put_down')
            self._log_duration_ms('create_put_down_task total (failed)', total_timer)
            return None
        
        self.strategy.get_logger().info(f'✅ Path generated: {len(stm32_commands)} waypoints')
        
        # Build task sequence with STM32 waypoints + ESP command
        task_sequence = []
        
        # Add STM32 waypoint commands
        last_cmd = None
        first_stm_command = True
        for cmd in stm32_commands:
            values = cmd.replace('go_to(', '').replace(')', '').split(',')
            x = float(values[0].strip())
            y = float(values[1].strip())
            speed = int(values[2].strip())
            
            if cmd != last_cmd:
                if device == 'ESP_back':
                    stm32_cmd = 'go_to_backwards'
                else:
                    stm32_cmd = 'go_to'
                
                task_sequence.append({
                    'device': 'STM',
                    'command': stm32_cmd,
                    'params': [x, y, 1100.0, {'from_pathfinding': True}],
                    'wait': first_stm_command
                })
                first_stm_command = False
            last_cmd = cmd
        
        # Add final orientation
        goal_angle = self.pathfinder.goal_angle
        if goal_angle is not None:
            # # Convert from radians to degrees
            goal_angle_deg = math.degrees(goal_angle)
            task_sequence.append({
                'device': 'STM',
                'command': 'orientate',
                'params': [goal_angle_deg, 0.0],  # [angle_in_degrees, is_radian=False]
                'wait': False
            })

        # Optional clearing sweep along putdown circle diameter:
        # target point -> opposite diameter point -> back to target point
        if center_x is not None and center_y is not None:
            opposite_x = (2.0 * center_x) - target_x
            opposite_y = (2.0 * center_y) - target_y

            first_cmd = 'go_to_backwards' if device == 'ESP_back' else 'go_to'
            second_cmd = 'go_to' if first_cmd == 'go_to_backwards' else 'go_to_backwards'

            task_sequence.append({
                'device': 'STM',
                'command': first_cmd,
                'params': [opposite_x, opposite_y, 1100.0, {'from_pathfinding': True, 'clear_zone_sweep': True}],
                'wait': False
            })
            task_sequence.append({
                'device': 'STM',
                'command': second_cmd,
                'params': [target_x, target_y, 1100.0, {'from_pathfinding': True, 'clear_zone_sweep': True}],
                'wait': False
            })

            self.strategy.get_logger().info(
                f'🧹 Added putdown clearing sweep via diameter: '
                f'({target_x:.0f},{target_y:.0f}) -> ({opposite_x:.0f},{opposite_y:.0f}) -> ({target_x:.0f},{target_y:.0f}) '
                f'using {first_cmd} then {second_cmd}'
            )
        
        # Add ESP put_down command
        task_sequence.append({
            'device': device,
            'command': 'put_down',
            'params': {},
            'wait': True
        })
        
        task = {
            'name': f'putdown_{device.split("_")[1]}',
            'sequence': task_sequence,
            'global_goal': {
                'x': target_x,
                'y': target_y,
                'theta': target_theta
            }
        }
        
        # NOTE: Picker status should be updated by the executor after successful put_down,
        # not here during task generation. Premature status updates can cause stale state
        # if the task fails or is interrupted.
        
        self.strategy.get_logger().info(f'✅ Generated put_down task: {task["name"]}')
        self._log_duration_ms('create_put_down_task total', total_timer)
        return task


# Example usage for testing
if __name__ == '__main__':
    print("DecisionMaker module - use with StrategyManager")
