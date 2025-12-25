"""
Hybrid A* Pathfinding Algorithm Implementation
==============================================

This module implements Hybrid A* for rectangular robots with orientation.
Allows the robot to get closer to obstacles by considering its actual footprint.
"""

import heapq
import numpy as np
import math
from typing import List, Tuple, Optional, Dict


class HybridAStarPathfinder:
    """
    Hybrid A* implementation for rectangular differential drive robots.
    Considers robot orientation for precise collision detection.
    """
    
    def __init__(self, grid_width: int, grid_height: int, 
                 robot_width: float = 350.0, robot_length: float = 300.0,
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
        
        # Permanent static obstacle areas (always blocked, never cleared)
        self.permanent_obstacles = []
        
        # Orientation discretization (36 = 10° resolution for speed)
        self.num_orientations = 36
        self.orientation_step = 2 * math.pi / self.num_orientations
        
        # Motion primitives (optimized for speed)
        self.step_sizes = [50.0]  # mm - single step size for speed
        self.turn_angles = [-30, 0, 30]  # degrees - reduced angles
        
        # Cost weights
        self.forward_cost = 1.0
        self.backward_cost = 5.0  # Heavily discourage backward motion
        self.turn_cost = 1.5
        self.direction_change_cost = 2.0
        
        # Precomputed footprint masks for each orientation
        self.footprints = {}
        self._precompute_footprints()
    
    def _precompute_footprints(self):
        """
        Precompute footprint masks for all discretized orientations.
        Each footprint is a list of (dx, dy) offsets in grid cells from robot center.
        This dramatically speeds up collision checking.
        """
        print(f"🔧 Precomputing footprints for {self.num_orientations} orientations...")
        
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
        print(f"✅ Footprints precomputed (avg {avg_cells:.1f} cells per orientation)")
    
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
        """
        grid_resolution = self.grid_resolution
        
        # Convert world coordinates to grid cells
        grid_x_start = int(x_min / grid_resolution)
        grid_y_start = int(y_min / grid_resolution)
        grid_x_end = int(x_max / grid_resolution) + 1
        grid_y_end = int(y_max / grid_resolution) + 1
        
        # Clamp to grid boundaries
        grid_x_start = max(0, min(grid_x_start, self.grid_width - 1))
        grid_x_end = max(0, min(grid_x_end, self.grid_width))
        grid_y_start = max(0, min(grid_y_start, self.grid_height - 1))
        grid_y_end = max(0, min(grid_y_end, self.grid_height))
        
        # Store the area definition
        area = {
            'x_min': x_min,
            'y_min': y_min,
            'x_max': x_max,
            'y_max': y_max,
            'grid_x_start': grid_x_start,
            'grid_y_start': grid_y_start,
            'grid_x_end': grid_x_end,
            'grid_y_end': grid_y_end
        }
        self.permanent_obstacles.append(area)
        
        # Immediately mark in grid
        self.grid[grid_y_start:grid_y_end, grid_x_start:grid_x_end] = True
        
        width = x_max - x_min
        height = y_max - y_min
        print(f"🚫 Added permanent obstacle area: ({x_min:.0f},{y_min:.0f}) to ({x_max:.0f},{y_max:.0f})")
        print(f"   Size: {width:.0f}mm × {height:.0f}mm")
        print(f"   Grid cells: [{grid_x_start}:{grid_x_end}, {grid_y_start}:{grid_y_end}]")
    
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
        
        # Re-apply permanent obstacle areas FIRST (always present)
        for area in self.permanent_obstacles:
            self.grid[area['grid_y_start']:area['grid_y_end'], 
                      area['grid_x_start']:area['grid_x_end']] = True
        
        # Add static walls if provided
        # Note: In ROS2 node mode, static walls are loaded from JSON file
        # For standalone use, can pass static_walls parameter or they default to empty
        if static_walls is None:
            static_walls = []
        
        # Mark static walls first
        for wall in static_walls:
            x_world = wall['x']
            y_world = wall['y']
            width = wall['width']
            height = wall['height']
            
            grid_x_start = int(x_world / grid_resolution)
            grid_y_start = int(y_world / grid_resolution)
            grid_x_end = int((x_world + width) / grid_resolution) + 1
            grid_y_end = int((y_world + height) / grid_resolution) + 1
            
            grid_x_start = max(0, min(grid_x_start, self.grid_width - 1))
            grid_x_end = max(0, min(grid_x_end, self.grid_width))
            self.grid[grid_y_start:grid_y_end, grid_x_start:grid_x_end] = True
        
        # Mark dynamic obstacles
        for obstacle in obstacles:
            x_world = obstacle['x']
            y_world = obstacle['y']
            width = obstacle['width']
            height = obstacle['height']
            
            # Convert to grid (no inflation)
            grid_x_start = int(x_world / grid_resolution)
            grid_y_start = int(y_world / grid_resolution)
            grid_x_end = int((x_world + width) / grid_resolution) + 1
            grid_y_end = int((y_world + height) / grid_resolution) + 1
            
            # Clamp to boundaries
            grid_x_start = max(0, min(grid_x_start, self.grid_width - 1))
            grid_x_end = max(0, min(grid_x_end, self.grid_width))
            grid_y_start = max(0, min(grid_y_start, self.grid_height - 1))
            grid_y_end = max(0, min(grid_y_end, self.grid_height))
            
            # Mark as occupied
            self.grid[grid_y_start:grid_y_end, grid_x_start:grid_x_end] = True
        
        num_static = len(static_walls) if static_walls else 0
        num_permanent = len(self.permanent_obstacles)
        print(f"✓ Set {len(obstacles)} dynamic obstacles + {num_static} static walls + {num_permanent} permanent areas")
    
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
        # Rotate-in-place primitives (stay at same position, just change orientation)
        # This allows the robot to turn without moving - critical for tight spaces
        for rotate_angle in [-90, 90]:  # degrees - just 90° turns for speed
            turn_rad = math.radians(rotate_angle)
            new_theta = theta + turn_rad
            new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))
            
            # Check if rotation is collision-free
            if not self.is_collision(x, y, new_theta, grid_resolution):
                # Small cost for rotation (just turning, no distance traveled)
                cost = self.turn_cost * abs(rotate_angle) * 0.5
                yield (x, y, new_theta, cost)
        
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
        return 1.2 * math.sqrt((x - goal_x)**2 + (y - goal_y)**2)
    
    def discretize_pose(self, x: float, y: float, theta: float, grid_resolution: float) -> Tuple[int, int, int]:
        """Discretize continuous pose for duplicate detection."""
        grid_x = int(x / grid_resolution)
        grid_y = int(y / grid_resolution)
        theta_idx = int((theta + math.pi) / self.orientation_step) % self.num_orientations
        return (grid_x, grid_y, theta_idx)
    
    def find_path(self, start_pose: Tuple[float, float, float], 
                  goal_pose: Tuple[float, float, float], 
                  grid_resolution: float,
                  max_iterations: int = 50000) -> Optional[List[Tuple[float, float, float]]]:
        """
        Find path using Hybrid A*.
        
        Args:
            start_pose: (x, y, theta) in mm and radians
            goal_pose: (x, y, theta) in mm and radians
            grid_resolution: Grid cell size in mm
            max_iterations: Max search iterations
            
        Returns:
            List of (x, y, theta) waypoints or None
        """
        start_x, start_y, start_theta = start_pose
        goal_x, goal_y, goal_theta = goal_pose
        
        print(f"🚀 Starting Hybrid A* search...")
        print(f"   Start: ({start_x:.1f}, {start_y:.1f}, {math.degrees(start_theta):.1f}°)")
        print(f"   Goal: ({goal_x:.1f}, {goal_y:.1f}, {math.degrees(goal_theta):.1f}°)")
        
        # Check start/goal validity
        if self.is_collision(start_x, start_y, start_theta, grid_resolution):
            print("✗ Start position is in collision!")
            return None
        
        if self.is_collision(goal_x, goal_y, goal_theta, grid_resolution):
            print("✗ Goal position is in collision!")
            return None
        
        # Priority queue: (f_score, counter, pose)
        counter = 0
        open_set = [(0, counter, start_pose)]
        counter += 1
        
        # Tracking
        came_from = {}
        g_score = {self.discretize_pose(*start_pose, grid_resolution): 0}
        closed_set = set()
        
        # Goal tolerance (relaxed for speed)
        goal_tolerance = 20.0  # mm
        angle_tolerance = math.radians(20)  # degrees
        
        iterations = 0
        while open_set and iterations < max_iterations:
            iterations += 1
            
            _, _, current = heapq.heappop(open_set)
            current_x, current_y, current_theta = current
            current_discrete = self.discretize_pose(current_x, current_y, current_theta, grid_resolution)
            
            # Check if reached goal
            dist_to_goal = math.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)
            angle_diff = abs(math.atan2(math.sin(current_theta - goal_theta), 
                                       math.cos(current_theta - goal_theta)))
            
            if dist_to_goal < goal_tolerance and angle_diff < angle_tolerance:
                print(f"✅ Path found in {iterations} iterations!")
                return self._reconstruct_path(came_from, current)
            
            if current_discrete in closed_set:
                continue
            closed_set.add(current_discrete)
            
            # Expand forward only (rotate-in-place handles orientation changes)
            for is_forward in [True]:
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
        
        print(f"✗ No path found after {iterations} iterations")
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
            Smoothed path
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
        
        return smoothed
    
    def _is_line_collision_free(self, pose1: Tuple[float, float, float], 
                                pose2: Tuple[float, float, float], 
                                grid_resolution: float) -> bool:
        """Check if line between two poses is collision-free."""
        x1, y1, theta1 = pose1
        x2, y2, theta2 = pose2
        
        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        num_checks = int(dist / (grid_resolution * 0.5))
        
        for i in range(num_checks + 1):
            t = i / max(num_checks, 1)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            theta = theta1 + t * (theta2 - theta1)
            
            if self.is_collision(x, y, theta, grid_resolution):
                return False
        
        return True
