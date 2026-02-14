#!/usr/bin/env python3
"""
Verify if path waypoints collide with permanent area
"""
import sys
import math
sys.path.append('/home/ubuntu/ROS2-WS-EURO/src/hybrid_astar')
from hybrid_astar.hybrid_astar import HybridAStarPathfinder

# Setup
map_width, map_height = 2000.0, 3000.0
grid_resolution = 15.0
robot_width, robot_length = 300.0, 300.0
safety_margin = 5.0

grid_width = int(map_width / grid_resolution)
grid_height = int(map_height / grid_resolution)
effective_width = robot_width + 2 * safety_margin
effective_length = robot_length + 2 * safety_margin

pathfinder = HybridAStarPathfinder(
    grid_width, grid_height, effective_width, effective_length, grid_resolution
)

# Add permanent area
pathfinder.add_permanent_obstacle_area(0.0, 600.0, 450.0, 2400.0)

# Set obstacles (recalculates permanent area with correct resolution)
pathfinder.set_obstacles([], grid_resolution, static_walls=None)

# Test waypoints from user's path
waypoints = [
    (343.5, 224.7, 0.0),    # Waypoint 1
    (636.8, 682.7, 60.0),   # Waypoint 2 - Now ABOVE boundary
    (587.7, 2560.6, 180.0), # Waypoint 3
    (291.9, 2738.0, 0.0),   # Waypoint 4
]

print("🔍 Checking waypoint collisions with permanent area (0,600)→(450,2400)\n")

for i, (x, y, theta_deg) in enumerate(waypoints, 1):
    theta_rad = math.radians(theta_deg)
    
    # Check collision
    collision = pathfinder.is_collision(x, y, theta_rad, grid_resolution)
    
    # Get corners
    corners = pathfinder.get_robot_corners(x, y, theta_rad)
    
    # Check which corners are in permanent area
    perm_area = {'x_min': 0, 'x_max': 450, 'y_min': 600, 'y_max': 2400}
    corners_in_perm = []
    
    for ci, (cx, cy) in enumerate(corners):
        in_perm = (perm_area['x_min'] <= cx <= perm_area['x_max'] and 
                   perm_area['y_min'] <= cy <= perm_area['y_max'])
        if in_perm:
            corners_in_perm.append((ci, cx, cy))
    
    status = "❌ COLLISION" if collision else "✅ OK"
    print(f"Waypoint {i}: ({x:.1f}, {y:.1f}) θ={theta_deg:.0f}° - {status}")
    
    if corners_in_perm:
        print(f"  ⚠️  {len(corners_in_perm)} corner(s) in permanent area:")
        for ci, cx, cy in corners_in_perm:
            print(f"     Corner {ci}: ({cx:.1f}, {cy:.1f})")
    
    # Show distance to permanent area edges
    min_dist_x = min(abs(x - perm_area['x_min']), abs(x - perm_area['x_max']))
    min_dist_y = min(abs(y - perm_area['y_min']), abs(y - perm_area['y_max']))
    
    if min_dist_x < 200 or min_dist_y < 200:  # Within 200mm
        print(f"  ⚠️  Close to permanent area: dist_x={min_dist_x:.1f}mm, dist_y={min_dist_y:.1f}mm")
    
    print()
