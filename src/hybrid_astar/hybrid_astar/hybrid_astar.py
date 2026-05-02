"""
Hybrid A* Pathfinding Algorithm Implementation
==============================================

This module implements Hybrid A* for rectangular robots with orientation.
Allows the robot to get closer to obstacles by considering its actual footprint.
"""

import heapq
import numpy as np
import math
import time
from typing import List, Tuple, Optional, Dict


class HybridAStarPathfinder:
    """
    Hybrid A* implementation for rectangular differential drive robots.
    Considers robot orientation for precise collision detection.
    """
    
    def __init__(self, grid_width: int, grid_height: int, 
                 robot_width: float = 300.0, robot_length: float = 300.0,
                 grid_resolution: float = 25.0):
        """
        Initialize Hybrid A* pathfinder.
        
        Args:
            grid_width: Grid width in cells
            grid_height: Grid height in cells
            robot_width: Robot width in mm (perpendicular to forward direction)
            robot_length: Robot length in mm (along forward direction)
            grid_resolution: Grid cell size in mm (needed for footprint precomputation)
        """
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.grid_resolution = grid_resolution
        
        # Binary occupancy grid
        self.grid = np.zeros((grid_height, grid_width), dtype=bool)

        # Debug metadata of occupied regions used to explain collision failures.
        # Updated on every set_obstacles() call.
        self.collision_sources: List[Dict[str, object]] = []
        
        # Permanent static obstacle areas (always blocked, never cleared)
        self.permanent_obstacles = []
        
        # Orientation discretization (16 = 22.5° resolution for speed)
        self.num_orientations = 16
        self.orientation_step = 2 * math.pi / self.num_orientations
        
        # Motion primitives (optimized for speed)
        self.step_sizes = [50.0]  # mm - larger steps for faster search
        self.turn_angles = [-30, 0, 30]  # degrees - left, straight, right
        
        # Cost weights
        self.forward_cost = 1.0
        self.backward_cost = 5.0  # Heavily discourage backward motion
        self.turn_cost = 1.0
        self.direction_change_cost = 2.0
        
        # Precomputed footprint masks for each orientation
        self.footprints = {}
        # Last planning-call diagnostics
        self.last_start_in_collision = False
        self.last_used_start_fallback = False
        self.last_used_reverse_fallback = False
        self.last_start_escape_mode = 'none'
        self.last_start_escape_distance_mm = 0.0
        self.last_used_bridge_fallback = False
        self.last_bridge_point = None
        # Keep internal planner logs quiet by default.
        # Main ROS node already logs request/response summaries and waypoints.
        self.verbose = False
        self._precompute_footprints()

    def _vprint(self, message: str):
        if self.verbose:
            print(message)

    def _is_collision_with_margin(
        self,
        x: float,
        y: float,
        theta: float,
        grid_resolution: float,
        margin_mm: float,
    ) -> bool:
        """Collision check for an expanded footprint (robot size + margin)."""
        if margin_mm <= 0.0:
            return self.is_collision(x, y, theta, grid_resolution)

        half_length = self.robot_length / 2.0 + margin_mm
        half_width = self.robot_width / 2.0 + margin_mm

        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        corners_local = [
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, -half_width),
            (-half_length, half_width),
        ]

        min_x = float("inf")
        max_x = float("-inf")
        min_y = float("inf")
        max_y = float("-inf")

        for lx, ly in corners_local:
            wx = x + lx * cos_theta - ly * sin_theta
            wy = y + lx * sin_theta + ly * cos_theta
            min_x = min(min_x, wx)
            max_x = max(max_x, wx)
            min_y = min(min_y, wy)
            max_y = max(max_y, wy)

        grid_x_start = int(math.floor(min_x / grid_resolution))
        grid_x_end = int(math.floor(max_x / grid_resolution))
        grid_y_start = int(math.floor(min_y / grid_resolution))
        grid_y_end = int(math.floor(max_y / grid_resolution))

        if (
            grid_x_start < 0
            or grid_y_start < 0
            or grid_x_end >= self.grid_width
            or grid_y_end >= self.grid_height
        ):
            return True

        for gx in range(grid_x_start, grid_x_end + 1):
            for gy in range(grid_y_start, grid_y_end + 1):
                cell_x = (gx + 0.5) * grid_resolution
                cell_y = (gy + 0.5) * grid_resolution

                # Point-in-expanded-rectangle test
                dx = cell_x - x
                dy = cell_y - y
                local_x = dx * cos_theta + dy * sin_theta
                local_y = -dx * sin_theta + dy * cos_theta

                if abs(local_x) <= half_length and abs(local_y) <= half_width:
                    if self.grid[gy, gx]:
                        return True

        return False

    def _find_directional_offset_free_pose(
        self,
        start_pose: Tuple[float, float, float],
        grid_resolution: float,
        direction: str,
        clearance_mm: float = 100.0,
        step_mm: float = 25.0,
        max_distance_mm: float = 2000.0,
        excluded_ids: Optional[set] = None,
    ) -> Optional[Tuple[Tuple[float, float, float], float]]:
        """
        Move strictly in the requested direction until a start pose is found that:
        1) is collision-free with a minimum obstacle clearance, and
        2) has at least one forward motion primitive,
        3) the straight-line segment from start to the candidate is clear
           (ignoring obstacles in excluded_ids, i.e. the ones the robot is
           already overlapping at the colliding start).

        Returns:
            ((x, y, theta), travel_distance_mm) or None
        """
        sx, sy, stheta = start_pose
        excluded_ids = excluded_ids or set()

        if direction == 'reverse':
            move_dx = -math.cos(stheta)
            move_dy = -math.sin(stheta)
        elif direction == 'forward':
            move_dx = math.cos(stheta)
            move_dy = math.sin(stheta)
        else:
            raise ValueError(f"Unsupported direction '{direction}' (expected 'reverse' or 'forward')")

        self._vprint(
            f"🔄 {direction.capitalize()} escape from ({sx:.1f}, {sy:.1f}) with min clearance {clearance_mm:.1f}mm..."
        )

        dist_mm = step_mm
        while dist_mm <= max_distance_mm + 1e-6:
            x = sx + dist_mm * move_dx
            y = sy + dist_mm * move_dy

            gx = int(x / grid_resolution)
            gy = int(y / grid_resolution)
            if gx < 0 or gx >= self.grid_width or gy < 0 or gy >= self.grid_height:
                dist_mm += step_mm
                continue

            # Require clearance at the destination (no exclusions — escape pose must be truly free)
            if self._is_collision_with_margin(x, y, stheta, grid_resolution, clearance_mm):
                dist_mm += step_mm
                continue

            # Candidate must also be able to move forward
            has_forward_move = False
            for _ in self.get_motion_primitives(x, y, stheta, grid_resolution, is_forward=True):
                has_forward_move = True
                break

            if not has_forward_move:
                dist_mm += step_mm
                continue

            # Verify that the straight-line path from start to this candidate is
            # free from all obstacles OTHER than the ones the start is already in.
            if not self._is_line_collision_free_excluding(
                start_pose, (x, y, stheta), grid_resolution, excluded_ids
            ):
                self._vprint(
                    f"  ✗ {direction.capitalize()} candidate at dist={dist_mm:.1f}mm blocked by obstacle on escape path"
                )
                dist_mm += step_mm
                continue

            self._vprint(
                f"✅ {direction.capitalize()} escape found: ({x:.1f}, {y:.1f}), dist={dist_mm:.1f}mm, clearance={clearance_mm:.1f}mm"
            )
            return ((x, y, stheta), dist_mm)

        self._vprint(
            f"⚠ {direction.capitalize()} escape failed up to {max_distance_mm:.1f}mm with {clearance_mm:.1f}mm clearance"
        )
        return None

    def _find_best_start_escape_pose(
        self,
        start_pose: Tuple[float, float, float],
        grid_resolution: float,
        clearance_mm: float = 100.0,
        step_mm: float = 25.0,
        max_distance_mm: float = 2000.0,
        excluded_ids: Optional[set] = None,
    ) -> Optional[Tuple[Tuple[float, float, float], str, float]]:
        """
        Try both reverse and forward escape for a colliding start and pick the easiest one.

        Selection rule: choose the valid candidate with minimum offset distance.
        Tie-breaker: prefer forward.

        Returns:
            (pose, direction, distance_mm) or None
        """
        candidates: List[Tuple[float, int, str, Tuple[float, float, float]]] = []

        for direction in ('reverse', 'forward'):
            result = self._find_directional_offset_free_pose(
                start_pose=start_pose,
                grid_resolution=grid_resolution,
                direction=direction,
                clearance_mm=clearance_mm,
                step_mm=step_mm,
                max_distance_mm=max_distance_mm,
                excluded_ids=excluded_ids,
            )
            if result is None:
                continue

            pose, distance_mm = result
            preference = 0 if direction == 'forward' else 1
            candidates.append((distance_mm, preference, direction, pose))

        if not candidates:
            return None

        candidates.sort(key=lambda item: (item[0], item[1]))
        best_distance_mm, _, best_direction, best_pose = candidates[0]
        return (best_pose, best_direction, best_distance_mm)

    def _find_radial_escape_pose(
        self,
        start_pose: Tuple[float, float, float],
        grid_resolution: float,
        clearance_mm: float = 100.0,
        step_mm: float = 50.0,
        max_distance_mm: float = 2200.0,
        preferred_min_y_mm: Optional[float] = 1500.0,
        excluded_ids: Optional[set] = None,
    ) -> Optional[Tuple[Tuple[float, float, float], float, str]]:
        """
        Omnidirectional fallback escape for colliding starts.

        Searches concentric rings around the original start and returns the first
        collision-free pose that:
          - has at least one forward primitive,
          - has a straight-line path from start that is clear of all obstacles
            except those in excluded_ids (the ones the start is already inside).
        Preference is given to candidates in y >= preferred_min_y_mm when provided.

        Returns:
            (pose, distance_mm, mode) where mode in {'radial_top', 'radial_any'}
        """
        sx, sy, stheta = start_pose
        excluded_ids = excluded_ids or set()

        angle_count = 24
        preferred_candidates: List[Tuple[float, Tuple[float, float, float]]] = []
        fallback_candidates: List[Tuple[float, Tuple[float, float, float]]] = []

        dist_mm = step_mm
        while dist_mm <= max_distance_mm + 1e-6:
            for i in range(angle_count):
                a = (2.0 * math.pi * i) / angle_count
                x = sx + dist_mm * math.cos(a)
                y = sy + dist_mm * math.sin(a)

                gx = int(x / grid_resolution)
                gy = int(y / grid_resolution)
                if gx < 0 or gx >= self.grid_width or gy < 0 or gy >= self.grid_height:
                    continue

                # Destination must be truly free (no exclusions)
                if self._is_collision_with_margin(x, y, stheta, grid_resolution, clearance_mm):
                    continue

                has_forward_move = False
                for _ in self.get_motion_primitives(x, y, stheta, grid_resolution, is_forward=True):
                    has_forward_move = True
                    break

                if not has_forward_move:
                    continue

                # Escape path must be clear (ignoring start's own obstacles)
                if not self._is_line_collision_free_excluding(
                    start_pose, (x, y, stheta), grid_resolution, excluded_ids
                ):
                    continue

                pose = (x, y, stheta)
                if preferred_min_y_mm is not None and y >= preferred_min_y_mm:
                    preferred_candidates.append((dist_mm, pose))
                else:
                    fallback_candidates.append((dist_mm, pose))

            if preferred_candidates:
                preferred_candidates.sort(key=lambda item: item[0])
                best_dist, best_pose = preferred_candidates[0]
                return (best_pose, best_dist, 'radial_top')

            if fallback_candidates:
                fallback_candidates.sort(key=lambda item: item[0])
                best_dist, best_pose = fallback_candidates[0]
                return (best_pose, best_dist, 'radial_any')

            dist_mm += step_mm

        return None

    def _path_length_mm(self, path: List[Tuple[float, float, float]]) -> float:
        if len(path) < 2:
            return 0.0
        total = 0.0
        for i in range(len(path) - 1):
            x1, y1, _ = path[i]
            x2, y2, _ = path[i + 1]
            total += math.hypot(x2 - x1, y2 - y1)
        return total

    def _generate_bridge_candidates(
        self,
        start_pose: Tuple[float, float, float],
        goal_pose: Tuple[float, float, float],
        grid_resolution: float,
        min_region_y_mm: float = 1500.0,
        stride_mm: float = 100.0,
        max_candidates: int = 60,
    ) -> List[Tuple[float, float, float]]:
        """Generate collision-free intermediate points in region y >= min_region_y_mm."""
        sx, sy, _ = start_pose
        gx, gy, _ = goal_pose

        map_width_mm = self.grid_width * grid_resolution
        map_height_mm = self.grid_height * grid_resolution

        x_min = grid_resolution
        x_max = max(grid_resolution, map_width_mm - grid_resolution)
        y_min = max(min_region_y_mm, grid_resolution)
        y_max = max(grid_resolution, map_height_mm - grid_resolution)

        if y_min > y_max:
            return []

        step = max(grid_resolution, stride_mm)

        scored: List[Tuple[float, Tuple[float, float, float]]] = []

        y = y_min
        while y <= y_max + 1e-6:
            x = x_min
            while x <= x_max + 1e-6:
                bridge_theta = math.atan2(gy - y, gx - x)

                # Keep candidates clearly outside occupied areas.
                if not self._is_collision_with_margin(x, y, bridge_theta, grid_resolution, margin_mm=50.0):
                    score = math.hypot(x - sx, y - sy) + math.hypot(gx - x, gy - y)
                    scored.append((score, (x, y, bridge_theta)))

                x += step
            y += step

        scored.sort(key=lambda item: item[0])
        return [pose for _, pose in scored[:max_candidates]]

    def _find_path_core(
        self,
        start_pose: Tuple[float, float, float],
        goal_pose: Tuple[float, float, float],
        grid_resolution: float,
        max_iterations: int,
        allow_goal_collision: bool,
        deadline: Optional[float] = None,
    ) -> Optional[List[Tuple[float, float, float]]]:
        """Core Hybrid A* search (assumes caller already handled start/goal validity policy)."""
        start_x, start_y, _ = start_pose
        goal_x, goal_y, goal_theta = goal_pose

        # Priority queue: (f_score, counter, pose)
        counter = 0
        open_set = [(0, counter, start_pose)]
        counter += 1

        # Tracking
        came_from = {}
        g_score = {self.discretize_pose(*start_pose, grid_resolution): 0}
        closed_set = set()

        # Goal tolerance (relaxed for speed)
        goal_tolerance = 50.0  # mm - increased for faster convergence

        iterations = 0
        while open_set and iterations < max_iterations:
            iterations += 1

            if deadline is not None and iterations % 500 == 0 and time.time() > deadline:
                return None

            _, _, current = heapq.heappop(open_set)
            current_x, current_y, current_theta = current
            current_discrete = self.discretize_pose(current_x, current_y, current_theta, grid_resolution)

            # Check if reached goal (position only, ignoring angle)
            dist_to_goal = math.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)

            if dist_to_goal < goal_tolerance:
                path = self._reconstruct_path(came_from, current)

                # ALWAYS ensure final waypoint is at exact goal position
                if len(path) > 0:
                    last_x, last_y, last_theta = path[-1]
                    final_dist = math.sqrt((last_x - goal_x)**2 + (last_y - goal_y)**2)

                    # Always replace last waypoint with exact goal (even if < 5mm away)
                    if final_dist > 0.1:  # Only if not already exactly at goal
                        # Check if goal position is collision-free
                        if not self.is_collision(goal_x, goal_y, goal_theta, grid_resolution):
                            # Check if line from last waypoint to goal is collision-free
                            if self._is_line_collision_free((last_x, last_y, last_theta),
                                                           (goal_x, goal_y, goal_theta),
                                                           grid_resolution):
                                # Replace last waypoint with exact goal
                                path[-1] = (goal_x, goal_y, goal_theta)
                            else:
                                # Path blocked - try adding as separate waypoint
                                if len(path) > 1:
                                    path[-1] = (goal_x, goal_y, goal_theta)
                        elif allow_goal_collision:
                            # Keep exact goal for overlap-allowed mode (batch pickup).
                            path[-1] = (goal_x, goal_y, goal_theta)

                return path

            if current_discrete in closed_set:
                continue
            closed_set.add(current_discrete)

            # Determine if robot should move forward or backward based on orientation to goal
            dx_to_goal = goal_x - current_x
            dy_to_goal = goal_y - current_y
            angle_to_goal = math.atan2(dy_to_goal, dx_to_goal)

            # Calculate angle difference (how much robot is facing away from goal)
            angle_diff = angle_to_goal - current_theta
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # If robot is facing significantly away from goal (> 90°), allow backward movement
            allow_backward = abs(angle_diff) > math.radians(90)

            # Expand both forward and (optionally) backward
            directions = [True]
            if allow_backward:
                directions.append(False)

            generated_neighbor = False

            for is_forward in directions:
                for new_x, new_y, new_theta, move_cost in self.get_motion_primitives(
                    current_x, current_y, current_theta, grid_resolution, is_forward
                ):
                    new_pose = (new_x, new_y, new_theta)
                    new_discrete = self.discretize_pose(new_x, new_y, new_theta, grid_resolution)

                    if new_discrete in closed_set:
                        continue

                    tentative_g = g_score[current_discrete] + move_cost

                    if new_discrete not in g_score or tentative_g < g_score[new_discrete]:
                        came_from[new_pose] = current
                        g_score[new_discrete] = tentative_g
                        f_score = tentative_g + self.heuristic(new_x, new_y, goal_x, goal_y)
                        heapq.heappush(open_set, (f_score, counter, new_pose))
                        counter += 1
                        generated_neighbor = True

            # Fallback: if no forward neighbor was possible, try backward primitives.
            if not generated_neighbor and not allow_backward:
                for new_x, new_y, new_theta, move_cost in self.get_motion_primitives(
                    current_x, current_y, current_theta, grid_resolution, False
                ):
                    new_pose = (new_x, new_y, new_theta)
                    new_discrete = self.discretize_pose(new_x, new_y, new_theta, grid_resolution)

                    if new_discrete in closed_set:
                        continue

                    tentative_g = g_score[current_discrete] + move_cost

                    if new_discrete not in g_score or tentative_g < g_score[new_discrete]:
                        came_from[new_pose] = current
                        g_score[new_discrete] = tentative_g
                        f_score = tentative_g + self.heuristic(new_x, new_y, goal_x, goal_y)
                        heapq.heappush(open_set, (f_score, counter, new_pose))
                        counter += 1

        return None

    def _find_two_stage_bridge_path(
        self,
        start_pose: Tuple[float, float, float],
        goal_pose: Tuple[float, float, float],
        grid_resolution: float,
        max_iterations: int,
        allow_goal_collision: bool,
        deadline: Optional[float] = None,
    ) -> Optional[List[Tuple[float, float, float]]]:
        """Try planning through a bridge point in the top region (y >= 1500mm)."""
        candidates = self._generate_bridge_candidates(
            start_pose=start_pose,
            goal_pose=goal_pose,
            grid_resolution=grid_resolution,
            min_region_y_mm=1500.0,
            stride_mm=100.0,
            max_candidates=60,
        )

        if not candidates:
            return None

        stage_iterations = max(5000, int(max_iterations // 2))
        best_path = None
        best_cost = float('inf')
        best_bridge = None

        for bridge_pose in candidates:
            if deadline is not None and time.time() > deadline:
                break

            path1 = self._find_path_core(
                start_pose=start_pose,
                goal_pose=bridge_pose,
                grid_resolution=grid_resolution,
                max_iterations=stage_iterations,
                allow_goal_collision=False,
                deadline=deadline,
            )
            if not path1:
                continue

            path2 = self._find_path_core(
                start_pose=bridge_pose,
                goal_pose=goal_pose,
                grid_resolution=grid_resolution,
                max_iterations=stage_iterations,
                allow_goal_collision=allow_goal_collision,
                deadline=deadline,
            )
            if not path2:
                continue

            merged = path1 + path2[1:]
            total_cost = self._path_length_mm(merged)
            if total_cost < best_cost:
                best_cost = total_cost
                best_path = merged
                best_bridge = bridge_pose

        if best_path is not None:
            self.last_used_bridge_fallback = True
            self.last_bridge_point = best_bridge

        return best_path
    
    def _precompute_footprints(self):
        """
        Precompute footprint masks for all discretized orientations.
        Each footprint is a list of (dx, dy) offsets in grid cells from robot center.
        This dramatically speeds up collision checking.
        """
        self._vprint(f"🔧 Precomputing footprints for {self.num_orientations} orientations...")
        
        for theta_idx in range(self.num_orientations):
            theta = -math.pi + theta_idx * self.orientation_step
            
            # Get robot corners in world coordinates (relative to center at origin)
            corners = self.get_robot_corners(0, 0, theta)
            
            # Get bounding box
            x_coords = [c[0] for c in corners]
            y_coords = [c[1] for c in corners]
            min_x, max_x = min(x_coords), max(x_coords)
            min_y, max_y = min(y_coords), max(y_coords)
            
            # Convert to grid cells (add padding)
            grid_min_x = int(math.floor(min_x / self.grid_resolution)) - 1
            grid_max_x = int(math.ceil(max_x / self.grid_resolution)) + 1
            grid_min_y = int(math.floor(min_y / self.grid_resolution)) - 1
            grid_max_y = int(math.ceil(max_y / self.grid_resolution)) + 1
            
            # Check all cells in bounding box
            footprint = []
            for dx_cell in range(grid_min_x, grid_max_x + 1):
                for dy_cell in range(grid_min_y, grid_max_y + 1):
                    # Cell center in world coordinates (relative to robot center)
                    cell_x = (dx_cell + 0.5) * self.grid_resolution
                    cell_y = (dy_cell + 0.5) * self.grid_resolution
                    
                    # Check if cell is inside robot footprint
                    if self.point_in_rectangle(cell_x, cell_y, 0, 0, theta):
                        footprint.append((dx_cell, dy_cell))
            
            # Store as Python list (MUCH faster iteration than NumPy array in Python loop!)
            self.footprints[theta_idx] = footprint
        
        avg_cells = sum(len(fp) for fp in self.footprints.values()) / len(self.footprints)
        self._vprint(f"✅ Footprints precomputed (avg {avg_cells:.1f} cells per orientation)")
    
    def add_permanent_obstacle_area(self, x_min: float, y_min: float, 
                                     x_max: float, y_max: float):
        """
        Add a permanent rectangular obstacle area that is ALWAYS blocked.
        This area is never cleared and persists across all pathfinding requests.
        
        Args:
            x_min, y_min: Bottom-left corner (mm)
            x_max, y_max: Top-right corner (mm)
        
        Example:
            # Block area from (0,600) to (450,2400)
            pathfinder.add_permanent_obstacle_area(0, 600, 450, 2400)
        
        Note:
            Grid indices are recalculated when set_obstacles() is called with the
            actual runtime grid_resolution, not stored at add time.
        """
        # Store the area definition (world coordinates only)
        # Grid indices will be calculated in set_obstacles() with correct resolution
        area = {
            'x_min': x_min,
            'y_min': y_min,
            'x_max': x_max,
            'y_max': y_max
        }
        self.permanent_obstacles.append(area)
        
        width = x_max - x_min
        height = y_max - y_min
        self._vprint(f"🚫 Added permanent obstacle area: ({x_min:.0f},{y_min:.0f}) to ({x_max:.0f},{y_max:.0f})")
        self._vprint(f"   Size: {width:.0f}mm × {height:.0f}mm")
        self._vprint(f"   ⚠️ Grid indices will be calculated when obstacles are set with actual resolution")
    
    def set_obstacles(self, obstacles: List[dict], grid_resolution: float, static_walls: List[dict] = None):
        """
        Mark obstacle cells in grid (NO inflation - we check exact robot footprint).
        Also adds static walls that are always present on the map.
        
        Args:
            obstacles: List of obstacle dicts with x, y, width, height
            grid_resolution: Grid cell size in mm
            static_walls: Optional list of static wall obstacles (if None, uses permanent areas)
        """
        # Clear grid
        self.grid.fill(False)
        self.collision_sources = []
        
        # Re-apply permanent obstacle areas FIRST with CORRECT grid_resolution
        for area in self.permanent_obstacles:
            # Calculate grid indices using the RUNTIME grid_resolution (not stored resolution)
            grid_x_start = int(area['x_min'] / grid_resolution)
            grid_y_start = int(area['y_min'] / grid_resolution)
            grid_x_end = int(area['x_max'] / grid_resolution) + 1
            grid_y_end = int(area['y_max'] / grid_resolution) + 1
            
            # Clamp to grid boundaries
            grid_x_start = max(0, min(grid_x_start, self.grid_width))
            grid_x_end = max(0, min(grid_x_end, self.grid_width))
            grid_y_start = max(0, min(grid_y_start, self.grid_height))
            grid_y_end = max(0, min(grid_y_end, self.grid_height))
            
            # Mark in grid
            self.grid[grid_y_start:grid_y_end, grid_x_start:grid_x_end] = True
            
            # Store calculated indices back for debugging
            area['grid_x_start'] = grid_x_start
            area['grid_y_start'] = grid_y_start
            area['grid_x_end'] = grid_x_end
            area['grid_y_end'] = grid_y_end

            self.collision_sources.append({
                'kind': 'permanent_area',
                'id': str(area.get('id', f"permanent_{len(self.collision_sources)}")),
                'x': (float(area['x_min']) + float(area['x_max'])) / 2.0,
                'y': (float(area['y_min']) + float(area['y_max'])) / 2.0,
                'width': float(area['x_max']) - float(area['x_min']),
                'height': float(area['y_max']) - float(area['y_min']),
                'grid_x_start': grid_x_start,
                'grid_y_start': grid_y_start,
                'grid_x_end': grid_x_end,
                'grid_y_end': grid_y_end,
            })
        
        # Add static walls if provided
        # Note: In ROS2 node mode, static walls are loaded from JSON file
        # For standalone use, can pass static_walls parameter or they default to empty
        if static_walls is None:
            static_walls = []
        
        # Mark static walls first (x/y are CENTER positions)
        for wall_idx, wall in enumerate(static_walls):
            x_world = wall['x']  # center X
            y_world = wall['y']  # center Y
            width = wall['width']
            height = wall['height']
            
            grid_x_start = int((x_world - width / 2.0) / grid_resolution)
            grid_y_start = int((y_world - height / 2.0) / grid_resolution)
            grid_x_end = int((x_world + width / 2.0) / grid_resolution) + 1
            grid_y_end = int((y_world + height / 2.0) / grid_resolution) + 1
            
            grid_x_start = max(0, min(grid_x_start, self.grid_width - 1))
            grid_x_end = max(0, min(grid_x_end, self.grid_width))
            grid_y_start = max(0, min(grid_y_start, self.grid_height - 1))
            grid_y_end = max(0, min(grid_y_end, self.grid_height))
            self.grid[grid_y_start:grid_y_end, grid_x_start:grid_x_end] = True

            self.collision_sources.append({
                'kind': 'static_wall',
                'id': str(wall.get('id', f"static_wall_{wall_idx}")),
                'x': float(x_world),
                'y': float(y_world),
                'width': float(width),
                'height': float(height),
                'grid_x_start': grid_x_start,
                'grid_y_start': grid_y_start,
                'grid_x_end': grid_x_end,
                'grid_y_end': grid_y_end,
            })
        
        # Mark dynamic obstacles (x/y are CENTER positions)
        for obstacle_idx, obstacle in enumerate(obstacles):
            x_world = obstacle['x']  # center X
            y_world = obstacle['y']  # center Y
            width = obstacle['width']
            height = obstacle['height']
            
            # Convert to grid using center ± half-size
            grid_x_start = int((x_world - width / 2.0) / grid_resolution)
            grid_y_start = int((y_world - height / 2.0) / grid_resolution)
            grid_x_end = int((x_world + width / 2.0) / grid_resolution) + 1
            grid_y_end = int((y_world + height / 2.0) / grid_resolution) + 1
            
            # Clamp to boundaries
            grid_x_start = max(0, min(grid_x_start, self.grid_width - 1))
            grid_x_end = max(0, min(grid_x_end, self.grid_width))
            grid_y_start = max(0, min(grid_y_start, self.grid_height - 1))
            grid_y_end = max(0, min(grid_y_end, self.grid_height))
            
            # Mark as occupied
            self.grid[grid_y_start:grid_y_end, grid_x_start:grid_x_end] = True

            self.collision_sources.append({
                'kind': 'dynamic',
                'id': str(obstacle.get('id', f"dynamic_{obstacle_idx}")),
                'x': float(x_world),
                'y': float(y_world),
                'width': float(width),
                'height': float(height),
                'color': str(obstacle.get('color', 'unknown')),
                'grid_x_start': grid_x_start,
                'grid_y_start': grid_y_start,
                'grid_x_end': grid_x_end,
                'grid_y_end': grid_y_end,
            })
        
        num_static = len(static_walls) if static_walls else 0
        num_permanent = len(self.permanent_obstacles)
        self._vprint(f"✓ Set {len(obstacles)} dynamic obstacles + {num_static} static walls + {num_permanent} permanent areas")

    def get_collision_debug_info(
        self,
        x: float,
        y: float,
        theta: float,
        grid_resolution: float,
        max_items: int = 5,
    ) -> List[Dict[str, object]]:
        """Return a compact list of obstacles that collide with robot footprint at pose."""
        theta_idx = int((theta + math.pi) / self.orientation_step) % self.num_orientations
        footprint = self.footprints[theta_idx]
        if len(footprint) == 0:
            return []

        center_gx = int(x / grid_resolution)
        center_gy = int(y / grid_resolution)

        results: List[Dict[str, object]] = []
        seen_keys = set()

        for dx_cell, dy_cell in footprint:
            gx = center_gx + dx_cell
            gy = center_gy + dy_cell

            if gx < 0 or gx >= self.grid_width or gy < 0 or gy >= self.grid_height:
                key = ('boundary', gx, gy)
                if key not in seen_keys:
                    seen_keys.add(key)
                    results.append({'kind': 'boundary', 'id': 'map_boundary', 'grid_cell': [gx, gy]})
                if len(results) >= max_items:
                    break
                continue

            if not self.grid[gy, gx]:
                continue

            matched = False
            for src in self.collision_sources:
                if (
                    int(src.get('grid_x_start', 0)) <= gx < int(src.get('grid_x_end', 0))
                    and int(src.get('grid_y_start', 0)) <= gy < int(src.get('grid_y_end', 0))
                ):
                    key = (src.get('kind', 'unknown'), src.get('id', 'unknown'))
                    if key in seen_keys:
                        continue
                    seen_keys.add(key)
                    entry = {
                        'kind': src.get('kind', 'unknown'),
                        'id': src.get('id', 'unknown'),
                        'x': src.get('x', None),
                        'y': src.get('y', None),
                        'width': src.get('width', None),
                        'height': src.get('height', None),
                        'grid_cell': [gx, gy],
                    }
                    if 'color' in src:
                        entry['color'] = src.get('color')
                    results.append(entry)
                    matched = True
                    if len(results) >= max_items:
                        break
            if len(results) >= max_items:
                break

            if not matched:
                key = ('occupied_cell', gx, gy)
                if key not in seen_keys:
                    seen_keys.add(key)
                    results.append({'kind': 'occupied_cell', 'id': 'unknown_obstacle', 'grid_cell': [gx, gy]})
                if len(results) >= max_items:
                    break

        return results
    
    def _get_start_excluded_sources(
        self,
        x: float,
        y: float,
        theta: float,
        grid_resolution: float,
    ) -> set:
        """
        Return the set of collision-source IDs to ignore when checking the escape
        path from a colliding start pose.

        For each obstacle the robot overlaps at (x, y, theta), derive its batch
        prefix (everything up to and including the last '_' before 'member', e.g.
        'sx_s5_member1' → prefix 'sx_s5_') and exclude all sources whose ID
        starts with that prefix.  Obstacles without 'member' in their ID are
        excluded individually.
        """
        colliding = self.get_collision_debug_info(x, y, theta, grid_resolution, max_items=50)
        excluded_ids: set = set()
        batch_prefixes: set = set()

        for info in colliding:
            src_id = info['id']
            excluded_ids.add(src_id)
            if 'member' in src_id:
                # e.g. 'sx_s5_member1' → prefix = 'sx_s5_'
                prefix = src_id.rsplit('member', 1)[0]
                if prefix:
                    batch_prefixes.add(prefix)

        # Expand to all sources whose ID starts with a matching batch prefix
        for src in self.collision_sources:
            sid = src['id']
            for prefix in batch_prefixes:
                if sid.startswith(prefix):
                    excluded_ids.add(sid)
                    break

        return excluded_ids

    def _is_collision_excluding(
        self,
        x: float,
        y: float,
        theta: float,
        grid_resolution: float,
        excluded_ids: set,
    ) -> bool:
        """
        Collision check that treats grid cells belonging *exclusively* to excluded
        sources as free.  Any cell that is also covered by a non-excluded source
        (or an unknown occupied cell) is still considered a collision.
        """
        if not excluded_ids:
            return self.is_collision(x, y, theta, grid_resolution)

        theta_idx = int((theta + math.pi) / self.orientation_step) % self.num_orientations
        footprint = self.footprints[theta_idx]

        center_gx = int(x / grid_resolution)
        center_gy = int(y / grid_resolution)

        for dx_cell, dy_cell in footprint:
            gx = center_gx + dx_cell
            gy = center_gy + dy_cell

            if gx < 0 or gx >= self.grid_width or gy < 0 or gy >= self.grid_height:
                return True  # boundary = collision

            if not self.grid[gy, gx]:
                continue

            # Determine whether every source covering this cell is excluded
            cell_has_non_excluded = False
            cell_matched = False
            for src in self.collision_sources:
                if (
                    int(src.get('grid_x_start', 0)) <= gx < int(src.get('grid_x_end', 0))
                    and int(src.get('grid_y_start', 0)) <= gy < int(src.get('grid_y_end', 0))
                ):
                    cell_matched = True
                    if src['id'] not in excluded_ids:
                        cell_has_non_excluded = True
                        break

            if not cell_matched:
                # Cell is occupied but not attributed to any known source → real collision
                return True

            if cell_has_non_excluded:
                return True

        return False

    def _is_line_collision_free_excluding(
        self,
        pose1: Tuple[float, float, float],
        pose2: Tuple[float, float, float],
        grid_resolution: float,
        excluded_ids: set,
    ) -> bool:
        """Line collision check that ignores cells belonging only to excluded sources."""
        x1, y1, theta1 = pose1
        x2, y2, theta2 = pose2
        dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        num_checks = max(10, int(dist / 5.0))
        for i in range(num_checks + 1):
            t = i / max(num_checks, 1)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            theta = theta1 + t * (theta2 - theta1)
            if self._is_collision_excluding(x, y, theta, grid_resolution, excluded_ids):
                return False
        return True

    def get_robot_corners(self, x: float, y: float, theta: float) -> List[Tuple[float, float]]:
        """
        Get the 4 corners of the robot's rectangular footprint.
        
        Args:
            x, y: Robot center position (mm)
            theta: Robot orientation (radians)
            
        Returns:
            List of 4 corner positions (x, y)
        """
        half_length = self.robot_length / 2
        half_width = self.robot_width / 2
        
        # Corners in robot's local frame
        corners_local = [
            (half_length, half_width),    # front-right
            (half_length, -half_width),   # front-left
            (-half_length, -half_width),  # back-left
            (-half_length, half_width)    # back-right
        ]
        
        # Transform to world frame
        corners_world = []
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        
        for lx, ly in corners_local:
            wx = x + lx * cos_theta - ly * sin_theta
            wy = y + lx * sin_theta + ly * cos_theta
            corners_world.append((wx, wy))
        
        return corners_world
    
    def is_collision(self, x: float, y: float, theta: float, grid_resolution: float) -> bool:
        """
        Optimized collision check using precomputed footprint masks.
        Early-exit on first collision for maximum speed.
        
        Args:
            x, y: Robot center position (mm)
            theta: Robot orientation (radians)
            grid_resolution: Grid cell size (mm)
            
        Returns:
            True if collision, False otherwise
        """
        # Discretize orientation to get footprint mask
        theta_idx = int((theta + math.pi) / self.orientation_step) % self.num_orientations
        
        # Get footprint offsets (numpy array)
        footprint = self.footprints[theta_idx]
        
        if len(footprint) == 0:
            return False
        
        # Get robot center in grid coordinates (use integer division for speed)
        center_gx = int(x / grid_resolution)
        center_gy = int(y / grid_resolution)
        
        # Check all cells in precomputed footprint with early exit
        for dx_cell, dy_cell in footprint:
            gx = center_gx + dx_cell
            gy = center_gy + dy_cell
            
            # Check boundaries (early exit)
            if gx < 0 or gx >= self.grid_width or gy < 0 or gy >= self.grid_height:
                return True
            
            # Check occupancy (early exit)
            if self.grid[gy, gx]:
                return True
        
        return False
    
    def point_in_rectangle(self, px: float, py: float, rect_x: float, rect_y: float, rect_theta: float) -> bool:
        """
        Check if point (px, py) is inside oriented rectangle.
        
        Args:
            px, py: Point coordinates
            rect_x, rect_y: Rectangle center
            rect_theta: Rectangle orientation (radians)
            
        Returns:
            True if point is inside rectangle
        """
        # Transform point to rectangle's local frame
        cos_theta = math.cos(-rect_theta)
        sin_theta = math.sin(-rect_theta)
        
        dx = px - rect_x
        dy = py - rect_y
        
        local_x = dx * cos_theta - dy * sin_theta
        local_y = dx * sin_theta + dy * cos_theta
        
        # Check if inside rectangle bounds
        half_length = self.robot_length / 2
        half_width = self.robot_width / 2
        
        return abs(local_x) <= half_length and abs(local_y) <= half_width
    
    def get_motion_primitives(self, x: float, y: float, theta: float, 
                              grid_resolution: float, is_forward: bool = True):
        """
        Generate motion primitives from current pose.
        
        Args:
            x, y, theta: Current pose
            grid_resolution: Grid cell size
            is_forward: Direction (True=forward, False=backward)
            
        Yields:
            (new_x, new_y, new_theta, cost)
        """
        # Rotate-in-place primitives disabled for speed
        # The turn_angles in movement primitives provide sufficient turning ability
        
        # Movement primitives (move with optional turn)
        direction = 1 if is_forward else -1
        base_cost = self.forward_cost if is_forward else self.backward_cost
        
        for step_size in self.step_sizes:
            for turn_angle in self.turn_angles:
                # Calculate new pose
                turn_rad = math.radians(turn_angle)
                new_theta = theta + turn_rad
                
                # Normalize angle to [-pi, pi]
                new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))
                
                new_x = x + direction * step_size * math.cos(new_theta)
                new_y = y + direction * step_size * math.sin(new_theta)
                
                # Check collision
                if not self.is_collision(new_x, new_y, new_theta, grid_resolution):
                    # Calculate cost
                    cost = base_cost * step_size
                    if abs(turn_angle) > 0:
                        cost += self.turn_cost * abs(turn_angle)
                    
                    yield (new_x, new_y, new_theta, cost)
    
    def heuristic(self, x: float, y: float, goal_x: float, goal_y: float) -> float:
        """Euclidean distance heuristic (weighted for faster convergence)."""
        return 1.5 * math.sqrt((x - goal_x)**2 + (y - goal_y)**2)
    
    def discretize_pose(self, x: float, y: float, theta: float, grid_resolution: float) -> Tuple[int, int, int]:
        """Discretize continuous pose for duplicate detection."""
        grid_x = int(x / grid_resolution)
        grid_y = int(y / grid_resolution)
        theta_idx = int((theta + math.pi) / self.orientation_step) % self.num_orientations
        return (grid_x, grid_y, theta_idx)
    
    def find_path(self, start_pose: Tuple[float, float, float],
                  goal_pose: Tuple[float, float, float],
                  grid_resolution: float,
                  max_iterations: int = 50000,
                  allow_goal_collision: bool = False,
                  time_limit_sec: float = 25.0) -> Optional[List[Tuple[float, float, float]]]:
        """
        Find path using Hybrid A*.

        Args:
            start_pose: (x, y, theta) in mm and radians
            goal_pose: (x, y, theta) in mm and radians
            grid_resolution: Grid cell size in mm
            max_iterations: Max search iterations
            allow_goal_collision: If True, allows goal to be in collision (for batch pickup)
            time_limit_sec: Wall-clock budget for the entire search (default 25s to stay under
                            the 30s service timeout in pathfinder_controller)

        Returns:
            List of (x, y, theta) waypoints or None
        """
        deadline = time.time() + time_limit_sec if time_limit_sec > 0 else None

        # Reset per-call diagnostics
        self.last_start_in_collision = False
        self.last_used_start_fallback = False
        self.last_used_reverse_fallback = False
        self.last_start_escape_mode = 'none'
        self.last_start_escape_distance_mm = 0.0
        self.last_used_bridge_fallback = False
        self.last_bridge_point = None

        start_x, start_y, start_theta = start_pose
        goal_x, goal_y, goal_theta = goal_pose
        original_start_pose = start_pose
        effective_start_pose = start_pose
        escape_prefix_path: Optional[List[Tuple[float, float, float]]] = None
        
        self._vprint(f"🚀 Starting Hybrid A* search...")
        self._vprint(f"   Start: ({start_x:.1f}, {start_y:.1f}, {math.degrees(start_theta):.1f}°)")
        self._vprint(f"   Goal: ({goal_x:.1f}, {goal_y:.1f}, {math.degrees(goal_theta):.1f}°)")
        if allow_goal_collision:
            self._vprint(f"   🎯 Goal collision allowed (batch pickup mode)")
        
        # Check start/goal validity
        start_collision = self.is_collision(start_x, start_y, start_theta, grid_resolution)
        if start_collision:
            self.last_start_in_collision = True
            self._vprint(f"✗ Start position ({start_x:.1f}, {start_y:.1f}) is in collision!")
            # Check which grid cell
            gx = int(start_x / grid_resolution)
            gy = int(start_y / grid_resolution)
            self._vprint(f"   Grid cell: ({gx}, {gy})")
            if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                self._vprint(f"   Grid occupied: {self.grid[gy, gx]}")
            for idx, info in enumerate(self.get_collision_debug_info(start_x, start_y, start_theta, grid_resolution), start=1):
                self._vprint(f"   Collision[{idx}]: {info}")

            # Identify which obstacles (and their batch siblings) the robot is already
            # overlapping — these will be ignored when checking the escape-path segment.
            start_excluded_ids = self._get_start_excluded_sources(
                start_x, start_y, start_theta, grid_resolution
            )
            if start_excluded_ids:
                self._vprint(f"   Ignoring {len(start_excluded_ids)} source(s) on escape path: {start_excluded_ids}")

            best_escape = self._find_best_start_escape_pose(
                start_pose,
                grid_resolution,
                clearance_mm=100.0,
                excluded_ids=start_excluded_ids,
            )
            if best_escape is None:
                radial_escape = self._find_radial_escape_pose(
                    start_pose,
                    grid_resolution,
                    clearance_mm=100.0,
                    preferred_min_y_mm=1500.0,
                    excluded_ids=start_excluded_ids,
                )
                if radial_escape is None:
                    self._vprint("✗ Could not find any start escape pose (forward/reverse/radial)")
                    return None

                relocated, escape_distance_mm, escape_mode = radial_escape
                effective_start_pose = relocated
                start_x, start_y, start_theta = relocated
                self.last_used_start_fallback = True
                self.last_used_reverse_fallback = False
                self.last_start_escape_mode = escape_mode
                self.last_start_escape_distance_mm = escape_distance_mm
                self._vprint(
                    f"↪ Relocated start via {escape_mode} escape ({escape_distance_mm:.1f}mm): "
                    f"({start_x:.1f}, {start_y:.1f}, {math.degrees(start_theta):.1f}°)"
                )
            else:
                relocated, escape_mode, escape_distance_mm = best_escape
                effective_start_pose = relocated
                start_x, start_y, start_theta = relocated
                self.last_used_start_fallback = True
                self.last_used_reverse_fallback = (escape_mode == 'reverse')
                self.last_start_escape_mode = escape_mode
                self.last_start_escape_distance_mm = escape_distance_mm
                self._vprint(
                    f"↪ Relocated start via {escape_mode} escape ({escape_distance_mm:.1f}mm): "
                    f"({start_x:.1f}, {start_y:.1f}, {math.degrees(start_theta):.1f}°)"
                )

            # Try to build a proper escape prefix path from the original (possibly
            # colliding) start to the relocated free pose using planner primitives.
            escape_prefix_path = self._find_path_core(
                start_pose=original_start_pose,
                goal_pose=effective_start_pose,
                grid_resolution=grid_resolution,
                max_iterations=max(3000, int(max_iterations // 3)),
                allow_goal_collision=False,
                deadline=deadline,
            )

            if escape_prefix_path is not None and len(escape_prefix_path) > 1:
                self._vprint(
                    f"✓ Planned escape prefix with {len(escape_prefix_path)} waypoint(s) to reach relocated start"
                )
            else:
                escape_prefix_path = None
                self._vprint(
                    "⚠ Could not plan explicit escape prefix; executor may use direct first hop to relocated start"
                )
        
        goal_collision = self.is_collision(goal_x, goal_y, goal_theta, grid_resolution)
        if goal_collision and not allow_goal_collision:
            self._vprint(f"✗ Goal position ({goal_x:.1f}, {goal_y:.1f}) is in collision!")
            # Check which grid cell
            gx = int(goal_x / grid_resolution)
            gy = int(goal_y / grid_resolution)
            self._vprint(f"   Grid cell: ({gx}, {gy})")
            if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                self._vprint(f"   Grid occupied: {self.grid[gy, gx]}")
            for idx, info in enumerate(self.get_collision_debug_info(goal_x, goal_y, goal_theta, grid_resolution), start=1):
                self._vprint(f"   Collision[{idx}]: {info}")
            return None
        elif goal_collision and allow_goal_collision:
            self._vprint(f"⚠ Goal position ({goal_x:.1f}, {goal_y:.1f}) is in collision, but proceeding (batch pickup mode)")
        
        # First try direct planning.
        path = self._find_path_core(
            start_pose=effective_start_pose,
            goal_pose=(goal_x, goal_y, goal_theta),
            grid_resolution=grid_resolution,
            max_iterations=max_iterations,
            allow_goal_collision=allow_goal_collision,
            deadline=deadline,
        )
        if path is not None:
            if escape_prefix_path is not None:
                path = escape_prefix_path + path[1:]
            self._vprint("✅ Path found with direct planning")
            return path

        # Fallback: try two-stage planning through a top-region bridge point.
        bridge_path = self._find_two_stage_bridge_path(
            start_pose=effective_start_pose,
            goal_pose=(goal_x, goal_y, goal_theta),
            grid_resolution=grid_resolution,
            max_iterations=max_iterations,
            allow_goal_collision=allow_goal_collision,
            deadline=deadline,
        )
        if bridge_path is not None:
            if escape_prefix_path is not None:
                bridge_path = escape_prefix_path + bridge_path[1:]
            bx, by, _ = self.last_bridge_point if self.last_bridge_point is not None else (float('nan'), float('nan'), 0.0)
            self._vprint(f"✅ Path found via bridge fallback in y>=1500mm region (bridge=({bx:.1f}, {by:.1f}))")
            return bridge_path

        self._vprint("✗ No path found (direct + bridge fallback failed)")
        return None
    
    def _reconstruct_path(self, came_from: dict, current: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """Reconstruct path from came_from dictionary."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def smooth_path(self, path: List[Tuple[float, float, float]], 
                   grid_resolution: float) -> List[Tuple[float, float, float]]:
        """
        Smooth path by removing unnecessary waypoints.
        
        Args:
            path: Original path
            grid_resolution: Grid cell size
            
        Returns:
            Smoothed path (always preserves exact final waypoint)
        """
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # Try to connect to furthest visible point
            for j in range(len(path) - 1, i, -1):
                if self._is_line_collision_free(path[i], path[j], grid_resolution):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                i += 1
        
        # Ensure the exact final waypoint is preserved
        if len(smoothed) > 0 and len(path) > 0:
            if smoothed[-1] != path[-1]:
                smoothed.append(path[-1])
        
        return smoothed
    
    def _is_line_collision_free(self, pose1: Tuple[float, float, float],
                                pose2: Tuple[float, float, float],
                                grid_resolution: float) -> bool:
        """Check if line between two poses is collision-free.

        Uses the actual travel angle (atan2 from pose1 to pose2) as the robot
        heading for all samples.  Interpolating the stored waypoint thetas would
        use stale discretised orientations that don't reflect how the robot
        actually travels, causing the footprint to miss obstacles at certain
        heading-transition points.
        """
        x1, y1, _ = pose1
        x2, y2, _ = pose2

        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if dist < 1e-6:
            return True

        travel_theta = math.atan2(y2 - y1, x2 - x1)
        num_checks = max(10, int(dist / 5.0))

        for i in range(num_checks + 1):
            t = i / num_checks
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)

            if self.is_collision(x, y, travel_theta, grid_resolution):
                return False

        return True