#!/usr/bin/env python3
"""
Verify the new path with 10mm grid resolution.
"""
import math

# New path waypoints from logs
waypoints = [
    (278.0, 247.5, 0.0),
    (552.1, 622.3, 60.0),
    (556.3, 2499.2, 150.0),
    (319.7, 2642.5, 0.0)
]

# Permanent obstacle area
perm_x_min, perm_y_min = 0, 600
perm_x_max, perm_y_max = 450, 2400

# Robot dimensions (with safety margin)
robot_width = 300 + 2*5  # 310mm
robot_length = 300 + 2*5  # 310mm

print("="*80)
print("PATH COLLISION VERIFICATION - New Path (10mm Resolution)")
print("="*80)
print(f"\nPermanent obstacle area: ({perm_x_min},{perm_y_min}) → ({perm_x_max},{perm_y_max})")
print(f"Robot: {robot_width}mm × {robot_length}mm")
print()

def check_waypoint_collision(x, y, theta_deg):
    """Check if robot footprint at (x,y,theta) collides with permanent area"""
    theta_rad = math.radians(theta_deg)
    cos_t = math.cos(theta_rad)
    sin_t = math.sin(theta_rad)
    
    # Half dimensions
    hw = robot_width / 2
    hl = robot_length / 2
    
    # 4 corners relative to center
    corners = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]
    
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
    x_overlap = not (max_x <= perm_x_min or min_x >= perm_x_max)
    y_overlap = not (max_y <= perm_y_min or min_y >= perm_y_max)
    collision = x_overlap and y_overlap
    
    return collision, min_x, max_x, min_y, max_y, world_corners

print(f"{'WP':<4} {'Position':<22} {'Theta':<8} {'X Range':<20} {'Y Range':<20} {'Status'}")
print("-"*80)

collision_count = 0
for i, (x, y, theta_deg) in enumerate(waypoints, 1):
    collision, min_x, max_x, min_y, max_y, corners = check_waypoint_collision(x, y, theta_deg)
    
    status = "❌ COLLISION" if collision else "✅ SAFE"
    if collision:
        collision_count += 1
    
    print(f"{i:<4} ({x:6.1f}, {y:7.1f})  {theta_deg:6.1f}°  [{min_x:6.1f}, {max_x:6.1f}]  [{min_y:6.1f}, {max_y:6.1f}]  {status}")
    
    if collision:
        print(f"     ⚠️  Overlaps with permanent area!")
        print(f"     Corners: {[(f'{cx:.1f}', f'{cy:.1f}') for cx, cy in corners]}")
        
        # Calculate clearances
        if min_x > perm_x_min and min_x < perm_x_max:
            print(f"     Left edge at x={min_x:.1f}mm is INSIDE permanent area [0, 450]")
        if max_x > perm_x_min and max_x < perm_x_max:
            print(f"     Right edge at x={max_x:.1f}mm is INSIDE permanent area [0, 450]")
        
        if min_y > perm_y_min and min_y < perm_y_max:
            print(f"     Bottom edge at y={min_y:.1f}mm is INSIDE permanent area [600, 2400]")
        if max_y > perm_y_min and max_y < perm_y_max:
            print(f"     Top edge at y={max_y:.1f}mm is INSIDE permanent area [600, 2400]")

print("\n" + "="*80)
print("SUMMARY")
print("="*80)
print(f"Total waypoints: {len(waypoints)}")
print(f"Safe waypoints: {len(waypoints) - collision_count} ✅")
print(f"Collision waypoints: {collision_count} ❌")

if collision_count > 0:
    print("\n⚠️  PATH HAS COLLISIONS!")
    print("\nPossible issues:")
    print("1. Smoothing may be creating waypoints that violate collision constraints")
    print("2. Collision detection during planning may not be checking correctly")
    print("3. The footprint mask precomputation may have errors")
else:
    print("\n✅ Path is collision-free!")

# Check line segments between waypoints
print("\n" + "="*80)
print("CHECKING LINE SEGMENTS BETWEEN WAYPOINTS")
print("="*80)

def check_line_segment(x1, y1, theta1, x2, y2, theta2, num_samples=20):
    """Check if line segment has any collisions"""
    collisions = []
    for i in range(num_samples + 1):
        t = i / num_samples
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        # Interpolate angle
        theta = theta1 + t * (theta2 - theta1)
        
        collision, min_x, max_x, min_y, max_y, _ = check_waypoint_collision(x, y, theta)
        if collision:
            collisions.append((x, y, theta, min_x, max_x, min_y, max_y))
    
    return collisions

segment_collision_count = 0
for i in range(len(waypoints) - 1):
    x1, y1, theta1 = waypoints[i]
    x2, y2, theta2 = waypoints[i+1]
    
    print(f"\nSegment {i+1}→{i+2}: ({x1:.1f},{y1:.1f},{theta1:.0f}°) → ({x2:.1f},{y2:.1f},{theta2:.0f}°)")
    
    collisions = check_line_segment(x1, y1, theta1, x2, y2, theta2, num_samples=50)
    
    if collisions:
        segment_collision_count += 1
        print(f"  ❌ {len(collisions)}/51 sample points have collisions!")
        print(f"  First collision at: ({collisions[0][0]:.1f}, {collisions[0][1]:.1f}, {collisions[0][2]:.1f}°)")
        print(f"    Bounding box: x=[{collisions[0][3]:.1f}, {collisions[0][4]:.1f}], y=[{collisions[0][5]:.1f}, {collisions[0][6]:.1f}]")
    else:
        print(f"  ✅ No collisions detected in segment")

print("\n" + "="*80)
print(f"Segments with collisions: {segment_collision_count}/{len(waypoints)-1}")
print("="*80)
