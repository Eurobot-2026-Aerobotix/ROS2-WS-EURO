#!/usr/bin/env python3
"""
Analyze why goal position (196.67, 2587.33, -180°) is in collision
"""

import sys
import os
import math

# Import directly
sys.path.insert(0, 'src/hybrid_astar/hybrid_astar')
from hybrid_astar import HybridAStarPathfinder

def analyze_goal_collision():
    """Analyze collision at goal position"""
    
    # Robot specs (with safety margin)
    robot_width = 350.0 + 2 * 10.0  # 370mm
    robot_length = 300.0 + 2 * 10.0  # 320mm
    
    # Map setup
    map_width = 2000.0  # mm (from your test_map.json)
    map_height = 3000.0  # mm
    grid_resolution = 25.0  # mm
    
    grid_width = int(map_width / grid_resolution)
    grid_height = int(map_height / grid_resolution)
    
    print("="*70)
    print("GOAL COLLISION ANALYSIS")
    print("="*70)
    print(f"Robot: {robot_width}mm (width) x {robot_length}mm (length)")
    print(f"Map: {map_width}mm x {map_height}mm")
    print(f"Grid: {grid_width}x{grid_height} @ {grid_resolution}mm")
    print("="*70)
    
    # Create pathfinder
    pathfinder = HybridAStarPathfinder(
        grid_width=grid_width,
        grid_height=grid_height,
        robot_width=robot_width,
        robot_length=robot_length
    )
    
    # Add permanent obstacle area
    print("\n🚫 Adding permanent obstacle area: (0, 600) → (450, 2400)")
    pathfinder.add_permanent_obstacle_area(0.0, 600.0, 450.0, 2400.0)
    
    # Add obstacles from test_map.json
    obstacles = []
    
    # Set obstacles
    pathfinder.set_obstacles(obstacles, grid_resolution)
    
    # Goal position
    goal_x = 196.67
    goal_y = 2597.33
    goal_theta_deg = -180.0
    goal_theta = math.radians(goal_theta_deg)
    
    print(f"\n{'='*70}")
    print("GOAL POSITION ANALYSIS")
    print(f"{'='*70}")
    print(f"Goal center: ({goal_x}, {goal_y})")
    print(f"Goal orientation: {goal_theta_deg}° (theta={goal_theta:.4f} rad)")
    
    # Get robot corners
    corners = pathfinder.get_robot_corners(goal_x, goal_y, goal_theta)
    print(f"\nRobot footprint corners:")
    for i, (cx, cy) in enumerate(corners):
        corner_names = ["front-right", "front-left", "back-left", "back-right"]
        print(f"  {corner_names[i]:12s}: ({cx:7.2f}, {cy:7.2f})")
    
    # Get bounding box
    x_coords = [c[0] for c in corners]
    y_coords = [c[1] for c in corners]
    
    robot_x_min = min(x_coords)
    robot_x_max = max(x_coords)
    robot_y_min = min(y_coords)
    robot_y_max = max(y_coords)
    
    print(f"\nRobot bounding box:")
    print(f"  X: [{robot_x_min:.2f}, {robot_x_max:.2f}]")
    print(f"  Y: [{robot_y_min:.2f}, {robot_y_max:.2f}]")
    
    # Check permanent area overlap
    perm_x_min, perm_x_max = 0.0, 450.0
    perm_y_min, perm_y_max = 600.0, 2400.0
    
    print(f"\n{'='*70}")
    print("PERMANENT AREA CHECK")
    print(f"{'='*70}")
    print(f"Permanent area:")
    print(f"  X: [{perm_x_min:.2f}, {perm_x_max:.2f}]")
    print(f"  Y: [{perm_y_min:.2f}, {perm_y_max:.2f}]")
    
    # Check overlap
    x_overlap = robot_x_max > perm_x_min and robot_x_min < perm_x_max
    y_overlap = robot_y_max > perm_y_min and robot_y_min < perm_y_max
    
    print(f"\nOverlap check:")
    print(f"  X overlap: {x_overlap}")
    print(f"    Robot X: [{robot_x_min:.2f}, {robot_x_max:.2f}]")
    print(f"    Perm  X: [{perm_x_min:.2f}, {perm_x_max:.2f}]")
    print(f"  Y overlap: {y_overlap}")
    print(f"    Robot Y: [{robot_y_min:.2f}, {robot_y_max:.2f}]")
    print(f"    Perm  Y: [{perm_y_min:.2f}, {perm_y_max:.2f}]")
    
    if x_overlap and y_overlap:
        print(f"\n⚠️  Robot footprint OVERLAPS with permanent area!")
        
        # Calculate overlap amounts
        x_overlap_amount = min(robot_x_max, perm_x_max) - max(robot_x_min, perm_x_min)
        y_overlap_amount = min(robot_y_max, perm_y_max) - max(robot_y_min, perm_y_min)
        print(f"    X overlap: {x_overlap_amount:.2f}mm")
        print(f"    Y overlap: {y_overlap_amount:.2f}mm")
    else:
        print(f"\n✅ No overlap with permanent area")
    
    # Check collision
    print(f"\n{'='*70}")
    print("COLLISION DETECTION")
    print(f"{'='*70}")
    
    is_collision = pathfinder.is_collision(goal_x, goal_y, goal_theta, grid_resolution)
    
    if is_collision:
        print(f"\n❌ COLLISION DETECTED at goal position")
        print(f"   The robot cannot be placed at ({goal_x:.2f}, {goal_y:.2f}, {goal_theta_deg}°)")
    else:
        print(f"\n✅ NO COLLISION at goal position")
        print(f"   The robot CAN be placed at ({goal_x:.2f}, {goal_y:.2f}, {goal_theta_deg}°)")
    
    # Grid cell analysis
    print(f"\n{'='*70}")
    print("GRID CELL ANALYSIS")
    print(f"{'='*70}")
    
    # Check which grid cells the robot occupies
    goal_grid_x = int(goal_x / grid_resolution)
    goal_grid_y = int(goal_y / grid_resolution)
    
    print(f"Goal center grid cell: ({goal_grid_x}, {goal_grid_y})")
    
    # Get footprint cells
    theta_idx = int((goal_theta + math.pi) / (2 * math.pi) * pathfinder.num_orientations) % pathfinder.num_orientations
    footprint = pathfinder.footprints[theta_idx]
    
    print(f"Orientation index: {theta_idx} (theta={goal_theta:.4f})")
    print(f"Footprint size: {len(footprint)} cells")
    
    # Check which cells are occupied
    occupied_cells = []
    for dx, dy in footprint:
        gx = goal_grid_x + dx
        gy = goal_grid_y + dy
        
        if 0 <= gx < grid_width and 0 <= gy < grid_height:
            if pathfinder.grid[gy, gx]:
                occupied_cells.append((gx, gy))
    
    print(f"\nOccupied cells in robot footprint: {len(occupied_cells)}")
    if occupied_cells:
        print("Collision cells (grid coordinates):")
        for gx, gy in occupied_cells[:10]:  # Show first 10
            wx = gx * grid_resolution
            wy = gy * grid_resolution
            print(f"  Grid ({gx:3d}, {gy:3d}) = World ({wx:7.1f}, {wy:7.1f})")
        if len(occupied_cells) > 10:
            print(f"  ... and {len(occupied_cells) - 10} more")
    
    # Check permanent area grid cells
    print(f"\n{'='*70}")
    print("PERMANENT AREA GRID CELLS")
    print(f"{'='*70}")
    perm_grid_x_start = int(0 / grid_resolution)
    perm_grid_y_start = int(600 / grid_resolution)
    perm_grid_x_end = int(450 / grid_resolution) + 1
    perm_grid_y_end = int(2400 / grid_resolution) + 1
    
    print(f"Permanent area grid range:")
    print(f"  X: [{perm_grid_x_start}, {perm_grid_x_end})")
    print(f"  Y: [{perm_grid_y_start}, {perm_grid_y_end})")
    print(f"  Y max cell world coord: {(perm_grid_y_end-1) * grid_resolution} - {perm_grid_y_end * grid_resolution}")
    
    print(f"\nRobot Y min grid cell: {int(robot_y_min / grid_resolution)}")
    print(f"Permanent Y max grid cell: {perm_grid_y_end - 1}")
    
    # Check map boundaries
    print(f"\n{'='*70}")
    print("MAP BOUNDARY CHECK")
    print(f"{'='*70}")
    print(f"Map boundaries:")
    print(f"  X: [0, {map_width}]")
    print(f"  Y: [0, {map_height}]")
    
    if robot_x_min < 0:
        print(f"  ⚠️  Robot extends {abs(robot_x_min):.2f}mm OUTSIDE left edge!")
    if robot_x_max > map_width:
        print(f"  ⚠️  Robot extends {robot_x_max - map_width:.2f}mm OUTSIDE right edge!")
    if robot_y_min < 0:
        print(f"  ⚠️  Robot extends {abs(robot_y_min):.2f}mm OUTSIDE bottom edge!")
    if robot_y_max > map_height:
        print(f"  ⚠️  Robot extends {robot_y_max - map_height:.2f}mm OUTSIDE top edge!")
    
    print(f"\n{'='*70}")
    
    # Suggest alternative goal
    if is_collision:
        print("\n💡 SUGGESTED FIX:")
        if robot_y_min < perm_y_max and robot_y_max > perm_y_max:
            suggested_y = perm_y_max + (robot_y_max - robot_y_min) / 2 + 50
            print(f"   Move goal ABOVE permanent area:")
            print(f"   New Y: {suggested_y:.2f}mm (currently {goal_y:.2f}mm)")
        elif robot_x_max > perm_x_max:
            suggested_x = perm_x_max + (robot_x_max - robot_x_min) / 2 + 50
            print(f"   Move goal RIGHT of permanent area:")
            print(f"   New X: {suggested_x:.2f}mm (currently {goal_x:.2f}mm)")
    
    print(f"{'='*70}\n")
    return is_collision


if __name__ == "__main__":
    collision = analyze_goal_collision()
    sys.exit(1 if collision else 0)
