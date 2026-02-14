#!/usr/bin/env python3
"""
Verify the path with 10mm grid resolution doesn't collide with permanent area.
Checks each waypoint with the robot's footprint at different orientations.
"""

# Path waypoints from logs
waypoints = [
    (239.5, 276.0, 0.0),
    (537.7, 592.5, 60.0),
    (737.7, 2273.7, 120.0),
    (511.2, 2491.1, 150.0),
    (323.7, 2606.1, 0.0)
]

# Permanent obstacle area
perm_x_min, perm_y_min = 0, 600
perm_x_max, perm_y_max = 450, 2400

# Robot dimensions (with safety margin)
robot_width = 300 + 2*5  # 310mm
robot_length = 300 + 2*5  # 310mm
robot_radius = ((robot_width/2)**2 + (robot_length/2)**2)**0.5

print("="*70)
print("PATH COLLISION VERIFICATION (10mm Grid Resolution)")
print("="*70)
print(f"\nPermanent obstacle area: ({perm_x_min},{perm_y_min}) → ({perm_x_max},{perm_y_max})")
print(f"Robot: {robot_width}mm × {robot_length}mm (effective with safety margin)")
print(f"Robot radius (diagonal): {robot_radius:.1f}mm")
print(f"\n{'Waypoint':<12} {'Position':<20} {'Theta':<10} {'Min X':<10} {'Max X':<10} {'Min Y':<10} {'Max Y':<10} {'Status'}")
print("-"*120)

import math

for i, (x, y, theta_deg) in enumerate(waypoints, 1):
    theta_rad = math.radians(theta_deg)
    
    # Calculate footprint corners at this orientation
    # For a rectangle at angle theta, we need to check all 4 corners
    cos_t = math.cos(theta_rad)
    sin_t = math.sin(theta_rad)
    
    # Half dimensions
    hw = robot_width / 2
    hl = robot_length / 2
    
    # 4 corners relative to center
    corners = [
        (-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)
    ]
    
    # Rotate corners and translate to world position
    world_corners = []
    for cx, cy in corners:
        wx = x + cx * cos_t - cy * sin_t
        wy = y + cx * sin_t + cy * cos_t
        world_corners.append((wx, wy))
    
    # Find bounding box
    min_x = min(c[0] for c in world_corners)
    max_x = max(c[0] for c in world_corners)
    min_y = min(c[1] for c in world_corners)
    max_y = max(c[1] for c in world_corners)
    
    # Check collision with permanent area
    # Two rectangles collide if they overlap in both X and Y
    x_overlap = not (max_x <= perm_x_min or min_x >= perm_x_max)
    y_overlap = not (max_y <= perm_y_min or min_y >= perm_y_max)
    collision = x_overlap and y_overlap
    
    # Calculate clearances
    clearance_x_min = min_x - perm_x_min if min_x > perm_x_min else "N/A (outside)"
    clearance_x_max = perm_x_max - max_x if max_x < perm_x_max else "N/A (outside)"
    clearance_y_min = min_y - perm_y_min if min_y > perm_y_min else "N/A (outside)"
    clearance_y_max = perm_y_max - max_y if max_y < perm_y_max else "N/A (outside)"
    
    status = "❌ COLLISION" if collision else "✅ SAFE"
    
    print(f"{i:2d}. ({x:6.1f}, {y:6.1f}) {theta_deg:6.1f}°  {min_x:7.1f}  {max_x:7.1f}  {min_y:7.1f}  {max_y:7.1f}  {status}")
    
    if collision:
        print(f"    ⚠️  Robot footprint overlaps with permanent area!")
        if isinstance(clearance_x_min, float):
            print(f"       X clearance from left: {clearance_x_min:.1f}mm")
        if isinstance(clearance_x_max, float):
            print(f"       X clearance from right: {clearance_x_max:.1f}mm")
        if isinstance(clearance_y_min, float):
            print(f"       Y clearance from bottom: {clearance_y_min:.1f}mm")
        if isinstance(clearance_y_max, float):
            print(f"       Y clearance from top: {clearance_y_max:.1f}mm")

print("\n" + "="*70)
print("SUMMARY")
print("="*70)
collisions = sum(1 for _, (x, y, theta_deg) in enumerate(waypoints, 1) 
                 if any([
                     # Check each waypoint for collision
                     not (max([x + (robot_width/2)*math.cos(math.radians(theta_deg)) - (robot_length/2)*math.sin(math.radians(theta_deg)),
                               x - (robot_width/2)*math.cos(math.radians(theta_deg)) - (robot_length/2)*math.sin(math.radians(theta_deg)),
                               x + (robot_width/2)*math.cos(math.radians(theta_deg)) + (robot_length/2)*math.sin(math.radians(theta_deg)),
                               x - (robot_width/2)*math.cos(math.radians(theta_deg)) + (robot_length/2)*math.sin(math.radians(theta_deg))]) <= perm_x_min 
                          or min([x + (robot_width/2)*math.cos(math.radians(theta_deg)) - (robot_length/2)*math.sin(math.radians(theta_deg)),
                                  x - (robot_width/2)*math.cos(math.radians(theta_deg)) - (robot_length/2)*math.sin(math.radians(theta_deg)),
                                  x + (robot_width/2)*math.cos(math.radians(theta_deg)) + (robot_length/2)*math.sin(math.radians(theta_deg)),
                                  x - (robot_width/2)*math.cos(math.radians(theta_deg)) + (robot_length/2)*math.sin(math.radians(theta_deg))]) >= perm_x_max)
                     and not (max([y + (robot_width/2)*math.sin(math.radians(theta_deg)) + (robot_length/2)*math.cos(math.radians(theta_deg)),
                                   y - (robot_width/2)*math.sin(math.radians(theta_deg)) + (robot_length/2)*math.cos(math.radians(theta_deg)),
                                   y + (robot_width/2)*math.sin(math.radians(theta_deg)) - (robot_length/2)*math.cos(math.radians(theta_deg)),
                                   y - (robot_width/2)*math.sin(math.radians(theta_deg)) - (robot_length/2)*math.cos(math.radians(theta_deg))]) <= perm_y_min 
                              or min([y + (robot_width/2)*math.sin(math.radians(theta_deg)) + (robot_length/2)*math.cos(math.radians(theta_deg)),
                                      y - (robot_width/2)*math.sin(math.radians(theta_deg)) + (robot_length/2)*math.cos(math.radians(theta_deg)),
                                      y + (robot_width/2)*math.sin(math.radians(theta_deg)) - (robot_length/2)*math.cos(math.radians(theta_deg)),
                                      y - (robot_width/2)*math.sin(math.radians(theta_deg)) - (robot_length/2)*math.cos(math.radians(theta_deg))]) >= perm_y_max)
                 ]))

# Simpler check
safe_count = 0
collision_count = 0
for i, (x, y, theta_deg) in enumerate(waypoints, 1):
    theta_rad = math.radians(theta_deg)
    cos_t = math.cos(theta_rad)
    sin_t = math.sin(theta_rad)
    hw = robot_width / 2
    hl = robot_length / 2
    corners = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]
    world_corners = [(x + cx*cos_t - cy*sin_t, y + cx*sin_t + cy*cos_t) for cx, cy in corners]
    min_x = min(c[0] for c in world_corners)
    max_x = max(c[0] for c in world_corners)
    min_y = min(c[1] for c in world_corners)
    max_y = max(c[1] for c in world_corners)
    x_overlap = not (max_x <= perm_x_min or min_x >= perm_x_max)
    y_overlap = not (max_y <= perm_y_min or min_y >= perm_y_max)
    collision = x_overlap and y_overlap
    if collision:
        collision_count += 1
    else:
        safe_count += 1

print(f"\nTotal waypoints: {len(waypoints)}")
print(f"Safe waypoints: {safe_count} ✅")
print(f"Collision waypoints: {collision_count} ❌")

if collision_count > 0:
    print("\n⚠️  WARNING: Path has collisions! The pathfinding algorithm should not have")
    print("   generated this path. There may be a bug in the collision detection.")
else:
    print("\n✅ All waypoints are collision-free! Path is safe to execute.")

print("="*70)
