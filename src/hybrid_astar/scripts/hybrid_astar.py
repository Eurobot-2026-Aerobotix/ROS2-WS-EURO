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
                 robot_width: float = 350.0, robot_length: float = 300.0):
        """
        Initialize Hybrid A* pathfinder.
        
        Args:
            grid_width: Grid width in cells
            grid_height: Grid height in cells
            robot_width: Robot width in mm (perpendicular to forward direction)
            robot_length: Robot length in mm (along forward direction)
        """
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.robot_width = robot_width
        self.robot_length = robot_length
        
        # Binary occupancy grid
        self.grid = np.zeros((grid_height, grid_width), dtype=bool)
        
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
    
    def set_obstacles(self, obstacles: List[dict], grid_resolution: float):
        """
        Mark obstacle cells in grid (NO inflation - we check exact robot footprint).
        
        Args:
            obstacles: List of obstacle dicts with x, y, width, height
            grid_resolution: Grid cell size in mm
        """
        self.grid.fill(False)
        
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
        
        print(f"✓ Set {len(obstacles)} obstacles (no inflation - using exact robot footprint)")
    
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
        Check if robot at (x, y, theta) collides with obstacles or boundaries.
        Uses oriented bounding box collision detection.
        
        Args:
            x, y: Robot center position (mm)
            theta: Robot orientation (radians)
            grid_resolution: Grid cell size (mm)
            
        Returns:
            True if collision, False otherwise
        """
        # Get robot corners
        corners = self.get_robot_corners(x, y, theta)
        
        # Check boundary collision
        for cx, cy in corners:
            if cx < 0 or cx >= self.grid_width * grid_resolution:
                return True
            if cy < 0 or cy >= self.grid_height * grid_resolution:
                return True
        
        # Get bounding box of robot footprint
        x_coords = [c[0] for c in corners]
        y_coords = [c[1] for c in corners]
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        # Convert to grid cells
        grid_min_x = int(min_x / grid_resolution)
        grid_min_y = int(min_y / grid_resolution)
        grid_max_x = int(max_x / grid_resolution)
        grid_max_y = int(max_y / grid_resolution)
        
        # Check all cells in bounding box
        for gx in range(grid_min_x, grid_max_x + 1):
            for gy in range(grid_min_y, grid_max_y + 1):
                if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                    if self.grid[gy, gx]:
                        # Cell is occupied, check if it's inside robot footprint
                        cell_x = (gx + 0.5) * grid_resolution
                        cell_y = (gy + 0.5) * grid_resolution
                        if self.point_in_rectangle(cell_x, cell_y, x, y, theta):
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
        return 2.0 * math.sqrt((x - goal_x)**2 + (y - goal_y)**2)
    
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
        
        # Goal tolerance (relaxed for speed, refinement phase will get us closer)
        goal_tolerance = 50.0  # mm
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
                path = self._reconstruct_path(came_from, current)
                
                # Refinement phase: try to get closer to exact goal
                if dist_to_goal > 10.0:  # Only refine if further than 10mm
                    refined = self._refine_final_position(path, goal_pose, grid_resolution)
                    if refined:
                        print(f"   🎯 Refined final position: {dist_to_goal:.1f}mm → {math.sqrt((refined[-1][0]-goal_x)**2 + (refined[-1][1]-goal_y)**2):.1f}mm")
                        return refined
                
                return path
            
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
    
    def _refine_final_position(self, path: List[Tuple[float, float, float]], 
                               goal_pose: Tuple[float, float, float],
                               grid_resolution: float) -> Optional[List[Tuple[float, float, float]]]:
        """Try to refine the final position to get closer to the exact goal."""
        if len(path) < 2:
            return None
        
        goal_x, goal_y, goal_theta = goal_pose
        last_x, last_y, last_theta = path[-1]
        
        # Try small steps toward goal
        for step_size in [5.0, 10.0, 15.0, 20.0]:  # mm
            dx = goal_x - last_x
            dy = goal_y - last_y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < step_size:
                # Direct step to goal
                if not self.is_collision(goal_x, goal_y, goal_theta, grid_resolution):
                    return path[:-1] + [(goal_x, goal_y, goal_theta)]
            else:
                # Intermediate step
                ratio = step_size / dist
                new_x = last_x + dx * ratio
                new_y = last_y + dy * ratio
                angle_to_goal = math.atan2(dy, dx)
                
                if not self.is_collision(new_x, new_y, angle_to_goal, grid_resolution):
                    # Check if we can then reach goal
                    if not self.is_collision(goal_x, goal_y, goal_theta, grid_resolution):
                        return path[:-1] + [(new_x, new_y, angle_to_goal), (goal_x, goal_y, goal_theta)]
        
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
