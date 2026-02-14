#!/usr/bin/env python3
"""Check if start and goal positions are in collision"""
import math

# Positions from logs
start_x, start_y = 1554.3, 1090.8
goal_x, goal_y = 807.9, 478.3

# Permanent area
perm_x_min, perm_y_min = 0, 600
perm_x_max, perm_y_max = 450, 2400

# Robot dimensions
robot_width = 310  # 300 + 2*5
robot_length = 310

def check_collision(x, y, theta_deg=0):
    """Check if robot at (x,y,theta) collides with permanent area"""
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
    
    return collision, min_x, max_x, min_y, max_y

print("="*70)
print("COLLISION CHECK FOR START AND GOAL")
print("="*70)
print(f"\nPermanent obstacle area: ({perm_x_min},{perm_y_min}) → ({perm_x_max},{perm_y_max})")
print(f"Robot: {robot_width}mm × {robot_length}mm\n")

# Check start
collision, min_x, max_x, min_y, max_y = check_collision(start_x, start_y)
print(f"START: ({start_x:.1f}, {start_y:.1f})")
print(f"  Bounding box: x=[{min_x:.1f}, {max_x:.1f}], y=[{min_y:.1f}, {max_y:.1f}]")
print(f"  Status: {'❌ COLLISION' if collision else '✅ SAFE'}")
if collision:
    print(f"  ⚠️  Start position is in collision with permanent area!")
print()

# Check goal
collision, min_x, max_x, min_y, max_y = check_collision(goal_x, goal_y)
print(f"GOAL: ({goal_x:.1f}, {goal_y:.1f})")
print(f"  Bounding box: x=[{min_x:.1f}, {max_x:.1f}], y=[{min_y:.1f}, {max_y:.1f}]")
print(f"  Status: {'❌ COLLISION' if collision else '✅ SAFE'}")
if collision:
    print(f"  ⚠️  Goal position is in collision with permanent area!")
print()

print("="*70)
