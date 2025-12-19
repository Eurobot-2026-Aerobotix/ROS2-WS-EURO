#!/usr/bin/env python3
"""
Hybrid A* Path Generator
========================
Generates paths using Hybrid A* with orientation awareness.
"""

import json
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from hybrid_astar import HybridAStarPathfinder


def generate_hybrid_path(json_file_path: str) -> bool:
    """
    Generate path using Hybrid A* and save to JSON.
    
    Args:
        json_file_path: Path to JSON map file
        
    Returns:
        True if successful
    """
    try:
        # Load data
        with open(json_file_path, 'r') as f:
            data = json.load(f)
        
        print("📁 Loading map data...")
        
        # Extract info
        map_info = data['map_info']
        obstacles = data['obstacles']
        start_point = data['start_point']
        goal_point = data['goal_point']
        
        grid_resolution = map_info['grid_resolution']
        grid_width = int(map_info['width'] / grid_resolution)
        grid_height = int(map_info['height'] / grid_resolution)
        
        print(f"🗺️  Map: {grid_width}x{grid_height} grid ({grid_resolution}mm resolution)")
        print(f"📍 Start: ({start_point['x']:.1f}, {start_point['y']:.1f})")
        print(f"🎯 Goal: ({goal_point['x']:.1f}, {goal_point['y']:.1f})")
        print(f"🚗 Robot: 350mm x 300mm")
        print(f"📦 Obstacles: {len(obstacles)}")
        
        # Load robot config
        config_path = os.path.join(os.path.dirname(__file__), 'config.json')
        with open(config_path, 'r') as f:
            config = json.load(f)
        
        robot_width = config['robot']['width_mm']
        robot_length = config['robot']['length_mm']
        safety_margin = config['robot'].get('safety_margin_mm', 0)
        
        # Add safety margin to robot dimensions for collision checking
        effective_width = robot_width + 2 * safety_margin
        effective_length = robot_length + 2 * safety_margin
        
        print(f"🛡️  Safety margin: {safety_margin}mm (effective size: {effective_width:.0f}x{effective_length:.0f}mm)")
        
        # Create pathfinder with inflated robot dimensions
        pathfinder = HybridAStarPathfinder(grid_width, grid_height, effective_width, effective_length)
        pathfinder.set_obstacles(obstacles, grid_resolution)
        
        # Get orientations from JSON (if available), otherwise calculate
        # Negate angles from JSON since they use left=positive convention
        if 'theta' in start_point:
            start_orientation = -math.radians(start_point['theta'])
        else:
            # Calculate start orientation (point toward goal)
            dx = goal_point['x'] - start_point['x']
            dy = goal_point['y'] - start_point['y']
            start_orientation = math.atan2(dy, dx)
        
        if 'theta' in goal_point:
            goal_orientation = -math.radians(goal_point['theta'])
        else:
            goal_orientation = 0.0  # Face east at goal by default
        
        start_pose = (start_point['x'], start_point['y'], start_orientation)
        goal_pose = (goal_point['x'], goal_point['y'], goal_orientation)
        
        print(f"🧭 Start orientation: {math.degrees(start_orientation):.1f}°")
        print(f"🧭 Goal orientation: {math.degrees(goal_orientation):.1f}°")
        print(f"\n🔍 Searching with Hybrid A*...")
        
        # Find path
        path = pathfinder.find_path(start_pose, goal_pose, grid_resolution)
        
        if path:
            print(f"✅ Found path: {len(path)} waypoints")
            
            # Smooth path
            smoothed = pathfinder.smooth_path(path, grid_resolution)
            print(f"🔄 Smoothed to {len(smoothed)} waypoints ({100*(1-len(smoothed)/len(path)):.1f}% reduction)")
            
            # Try to set the goal orientation if it's collision-free
            if len(smoothed) > 0:
                x, y, current_theta = smoothed[-1]
                
                print(f"  Checking goal rotation at ({x:.1f}, {y:.1f}): {math.degrees(current_theta):.1f}° → {math.degrees(goal_orientation):.1f}°")
                
                # Check multiple intermediate angles during rotation to ensure safe rotation
                angle_diff = goal_orientation - current_theta
                # Normalize to [-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                # Check if rotation path is collision-free
                rotation_safe = True
                num_rotation_checks = max(18, int(abs(angle_diff) / math.radians(5)))  # Check every ~5°
                for i in range(num_rotation_checks + 1):
                    ratio = i / num_rotation_checks
                    check_angle = current_theta + ratio * angle_diff
                    if pathfinder.is_collision(x, y, check_angle, grid_resolution):
                        print(f"    ⚠ Rotation collision at angle {math.degrees(check_angle):.1f}° (step {i}/{num_rotation_checks})")
                        rotation_safe = False
                        break
                
                if rotation_safe:
                    # Add intermediate waypoint for rotation if angle change is large
                    if abs(angle_diff) > math.radians(30):
                        # Keep position but change orientation gradually
                        smoothed.append((x, y, goal_orientation))
                        print(f"✓ Added rotation waypoint: {math.degrees(current_theta):.1f}° → {math.degrees(goal_orientation):.1f}°")
                    else:
                        smoothed[-1] = (x, y, goal_orientation)
                        print(f"✓ Final orientation set to goal: {math.degrees(goal_orientation):.1f}°")
                else:
                    print(f"⚠ Goal orientation {math.degrees(goal_orientation):.1f}° would cause collision during rotation, keeping {math.degrees(current_theta):.1f}°")
            
            # Validate that each waypoint can safely rotate to travel angle
            # The robot needs to rotate from its current orientation to point toward the next waypoint
            print(f"🔍 Validating travel rotations for {len(smoothed)} waypoints...")
            validated = []
            for i in range(len(smoothed)):
                x, y, theta = smoothed[i]
                print(f"  Checking waypoint {i+1}: ({x:.1f}, {y:.1f}) θ={math.degrees(theta):.1f}°")
                
                if i < len(smoothed) - 1:
                    # Calculate travel angle toward next waypoint
                    next_x, next_y, _ = smoothed[i+1]
                    travel_angle = math.atan2(next_y - y, next_x - x)
                    
                    # Check if robot can rotate from current theta to travel angle
                    angle_diff = travel_angle - theta
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                    
                    # Check intermediate angles during rotation
                    num_checks = max(10, int(abs(angle_diff) / math.radians(5)))
                    collision_found = False
                    for j in range(num_checks + 1):
                        ratio = j / num_checks
                        check_angle = theta + ratio * angle_diff
                        if pathfinder.is_collision(x, y, check_angle, grid_resolution):
                            print(f"  ⚠ Waypoint {i+1} collision: θ={math.degrees(theta):.1f}° → {math.degrees(travel_angle):.1f}° hits obstacle at {math.degrees(check_angle):.1f}°")
                            collision_found = True
                            break
                    
                    if collision_found:
                        # Try to find a safe orientation at this position
                        # Test angles in 5° increments
                        safe_angle = None
                        for test_deg in range(-180, 180, 5):
                            test_angle = math.radians(test_deg)
                            # Check if this angle is collision-free
                            if not pathfinder.is_collision(x, y, test_angle, grid_resolution):
                                # Check if we can rotate to travel angle from this angle
                                test_diff = travel_angle - test_angle
                                while test_diff > math.pi:
                                    test_diff -= 2 * math.pi
                                while test_diff < -math.pi:
                                    test_diff += 2 * math.pi
                                
                                test_safe = True
                                test_checks = max(10, int(abs(test_diff) / math.radians(5)))
                                for k in range(test_checks + 1):
                                    ratio = k / test_checks
                                    check_angle = test_angle + ratio * test_diff
                                    if pathfinder.is_collision(x, y, check_angle, grid_resolution):
                                        test_safe = False
                                        break
                                
                                if test_safe:
                                    safe_angle = test_angle
                                    break
                        
                        if safe_angle is not None:
                            print(f"    → Fixed: using θ={math.degrees(safe_angle):.1f}° instead")
                            validated.append((x, y, safe_angle))
                        else:
                            print(f"    → No safe angle found, keeping original")
                            validated.append((x, y, theta))
                    else:
                        validated.append((x, y, theta))
                else:
                    # Last waypoint, keep as is
                    validated.append((x, y, theta))
            
            smoothed = validated
            
            # Remove waypoints that are too close (rotation-only waypoints)
            # Keep waypoints that are far apart (needed for obstacle avoidance)
            print(f"🔧 Removing rotation-only waypoints...")
            optimized = [smoothed[0]]  # Always keep first waypoint
            
            i = 1
            while i < len(smoothed) - 1:
                prev_x, prev_y, prev_theta = optimized[-1]  # Use last kept waypoint
                curr_x, curr_y, curr_theta = smoothed[i]
                next_x, next_y, next_theta = smoothed[i+1]
                
                # Calculate distance from last kept waypoint to current
                dist_to_curr = math.sqrt((curr_x - prev_x)**2 + (curr_y - prev_y)**2)
                
                # If waypoint is very close (< 100mm), it's just a rotation point
                # Skip it and let the robot rotate at the last kept position
                if dist_to_curr < 100.0:
                    print(f"  Skipping waypoint {i+1}: too close to previous ({dist_to_curr:.1f}mm)")
                    i += 1
                    continue
                
                # Calculate angle from last kept waypoint to current
                angle_to_curr = math.atan2(curr_y - prev_y, curr_x - prev_x)
                # Calculate angle from current to next
                angle_to_next = math.atan2(next_y - curr_y, next_x - curr_x)
                
                # Check if the path is roughly straight (angles are similar)
                angle_diff = abs(math.atan2(math.sin(angle_to_next - angle_to_curr), 
                                           math.cos(angle_to_next - angle_to_curr)))
                
                # Check if orientations are similar between current and next
                theta_diff = abs(math.atan2(math.sin(next_theta - curr_theta), 
                                           math.cos(next_theta - curr_theta)))
                
                # Keep waypoint if path changes significantly:
                # - Path direction change > 15 degrees (not just 5)
                # - Robot orientation change > 20 degrees (allow small adjustments)
                if angle_diff > math.radians(15) or theta_diff > math.radians(20):
                    optimized.append(smoothed[i])
                else:
                    print(f"  Skipping waypoint {i+1}: small direction change (dir:{math.degrees(angle_diff):.1f}°, θ:{math.degrees(theta_diff):.1f}°)")
                
                i += 1
            
            if len(smoothed) > 0:
                optimized.append(smoothed[-1])  # Always keep last waypoint
            
            if len(optimized) < len(smoothed):
                print(f"🎯 Optimized to {len(optimized)} waypoints (removed {len(smoothed)-len(optimized)} rotation-only waypoints)")
                smoothed = optimized
            
            # Intersection optimization - DISABLED for safety
            # This was eliminating waypoints needed to avoid obstacles  
            print(f"ℹ️  Intersection optimization disabled - keeping all waypoints")
            
            # DISABLED: Find intersections between extended segments
            """
            # DEBUG
            # print(f"   Input waypoints before intersection optimization:")
            # for idx, (x, y, theta) in enumerate(smoothed):
            #     print(f"     {idx+1}. ({x:.1f}, {y:.1f}) θ={math.degrees(theta):.1f}°")
            extended = [smoothed[0]]  # Keep start
            i = 1
            
            while i < len(smoothed):
                if i >= len(smoothed) - 1:
                    extended.append(smoothed[i])
                    break
                
                # Try to find intersection with a segment from a future waypoint
                # This can eliminate intermediate waypoints
                found_intersection = False
                
                # Try looking ahead 2 or more waypoints
                for skip_ahead in range(2, min(5, len(smoothed) - i)):
                    if i + skip_ahead >= len(smoothed):
                        break
                    
                    prev_x, prev_y, prev_theta = extended[-1]
                    curr_x, curr_y, curr_theta = smoothed[i]
                    target_x, target_y, target_theta = smoothed[i + skip_ahead]
                    
                    # DEBUG - uncomment to see all attempts
                    # print(f"    Trying: extend from waypoint {len(extended)} through {i+1} to intersect with line from {i+skip_ahead+1}")
                    
                    # Direction from previous to current (extend this line forward)
                    dir1_x = curr_x - prev_x
                    dir1_y = curr_y - prev_y
                    len1 = math.sqrt(dir1_x**2 + dir1_y**2)
                    if len1 < 1.0:
                        continue
                    dir1_x /= len1
                    dir1_y /= len1
                    
                    # Direction from target backward (toward where it came from)
                    # We want to extend FROM target BACKWARD, not from current
                    if i + skip_ahead > 1:
                        # Get the waypoint before target to determine its incoming direction
                        before_target_idx = i + skip_ahead - 1
                        before_target_x, before_target_y, _ = smoothed[before_target_idx]
                        dir2_x = before_target_x - target_x
                        dir2_y = before_target_y - target_y
                    else:
                        # If no previous waypoint, use direction from current to target
                        dir2_x = curr_x - target_x
                        dir2_y = curr_y - target_y
                    
                    len2 = math.sqrt(dir2_x**2 + dir2_y**2)
                    if len2 < 1.0:
                        continue
                    dir2_x /= len2
                    dir2_y /= len2
                    
                    # Find intersection of two lines:
                    # Line 1: prev + t1 * dir1
                    # Line 2: target + t2 * dir2
                    # Solve: prev + t1*dir1 = target + t2*dir2
                    
                    # Using determinant method
                    denom = dir1_x * dir2_y - dir1_y * dir2_x
                    if abs(denom) < 0.001:  # Lines are parallel
                        continue
                    
                    dx = target_x - prev_x
                    dy = target_y - prev_y
                    
                    t1 = (dx * dir2_y - dy * dir2_x) / denom
                    t2 = (dx * dir1_y - dy * dir1_x) / denom
                    
                    # Check if intersection is ahead of prev (t1 > 0) and ahead of target (t2 > 0)
                    # We want to extend forward from prev and backward from target
                    if t1 < 10.0 or t2 < 10.0:  # Must extend at least 10mm
                        # print(f"      ✗ Skipped: t1={t1:.1f}, t2={t2:.1f} (need both >10mm)")
                        continue
                    
                    # Calculate intersection point
                    int_x = prev_x + t1 * dir1_x
                    int_y = prev_y + t1 * dir1_y
                    
                    # Check if intersection point is reasonable (not too far)
                    dist_from_prev = math.sqrt((int_x - prev_x)**2 + (int_y - prev_y)**2)
                    dist_from_target = math.sqrt((int_x - target_x)**2 + (int_y - target_y)**2)
                    
                    # Allow longer extensions but not absurdly long
                    if dist_from_prev > 3000.0 or dist_from_target > 3000.0:
                        # print(f"      ✗ Skipped: too far (d_prev={dist_from_prev:.1f}, d_target={dist_from_target:.1f})")
                        continue
                    
                    # Also check that the intersection is not beyond the original waypoints
                    # (i.e., between prev and somewhere past target)
                    total_original_dist = 0
                    for j in range(len(extended) - 1, i + skip_ahead):
                        if j + 1 < len(smoothed):
                            x1, y1, _ = smoothed[j] if j >= 0 else extended[-1]
                            x2, y2, _ = smoothed[j + 1]
                            total_original_dist += math.sqrt((x2-x1)**2 + (y2-y1)**2)
                    
                    new_dist = dist_from_prev + dist_from_target
                    # Only accept if the new path is not significantly longer
                    if new_dist > total_original_dist * 1.5:
                        # print(f"      ✗ Skipped: new path too long ({new_dist:.1f} > {total_original_dist*1.5:.1f})")
                        continue
                    
                    # Check if path from prev to intersection is collision-free
                    angle1 = math.atan2(int_y - prev_y, int_x - prev_x)
                    dist1 = math.sqrt((int_x - prev_x)**2 + (int_y - prev_y)**2)
                    num_checks1 = max(int(dist1 / 5.0), 20)
                    
                    # DEBUG
                    # print(f"      Checking path1: ({prev_x:.1f},{prev_y:.1f})@{math.degrees(prev_theta):.1f}° → ({int_x:.1f},{int_y:.1f}), travel_angle={math.degrees(angle1):.1f}°, Δθ={math.degrees(angle1-prev_theta):.1f}°, dist={dist1:.1f}mm")
                    
                    # First check if robot can rotate to the required angle at the starting point
                    if pathfinder.is_collision(prev_x, prev_y, angle1, grid_resolution):
                        # print(f"      ✗ Skipped: cannot rotate to travel angle at start point")
                        continue
                    
                    path1_clear = True
                    for t in range(num_checks1 + 1):
                        ratio = t / num_checks1
                        check_x = prev_x + ratio * (int_x - prev_x)
                        check_y = prev_y + ratio * (int_y - prev_y)
                        if pathfinder.is_collision(check_x, check_y, angle1, grid_resolution):
                            # print(f"        Collision at check {t}/{num_checks1}: ({check_x:.1f},{check_y:.1f})")
                            path1_clear = False
                            break
                    
                    if not path1_clear:
                        # print(f"      ✗ Skipped: path1 collision (prev→intersection)")
                        continue
                    
                    # Check if path from intersection to target is collision-free
                    angle2 = math.atan2(target_y - int_y, target_x - int_x)
                    dist2 = math.sqrt((target_x - int_x)**2 + (target_y - int_y)**2)
                    num_checks2 = max(int(dist2 / 5.0), 20)
                    
                    path2_clear = True
                    for t in range(num_checks2 + 1):
                        ratio = t / num_checks2
                        check_x = int_x + ratio * (target_x - int_x)
                        check_y = int_y + ratio * (target_y - int_y)
                        if pathfinder.is_collision(check_x, check_y, angle2, grid_resolution):
                            path2_clear = False
                            break
                    
                    if path2_clear:
                        # Found valid intersection!
                        print(f"  ✓ Found intersection at ({int_x:.1f}, {int_y:.1f}): eliminated waypoints {i+1}-{i+skip_ahead-1}, saving {skip_ahead-1} waypoint(s)")
                        extended.append((int_x, int_y, angle1))  # Use angle toward intersection
                        i = i + skip_ahead
                        found_intersection = True
                        break
                    # else:
                        # print(f"      ✗ Skipped: path2 collision (intersection→target)")
                
                if not found_intersection:
                    # No intersection found, keep current waypoint
                    extended.append(smoothed[i])
                    i += 1
            
            # Handle last waypoint
            if len(extended) == 0 or (extended[-1][0] != smoothed[-1][0] or extended[-1][1] != smoothed[-1][1]):
                extended.append(smoothed[-1])
            
            if len(extended) < len(smoothed):
                print(f"🚀 Intersection optimization: {len(extended)} waypoints (removed {len(smoothed)-len(extended)} by finding intersections)")
                smoothed = extended
            """
            
            # Axis-aligned projection - DISABLED for safety
            print(f"ℹ️  Axis-aligned projection disabled - keeping all waypoints")
            
            # DISABLED: axis-aligned optimization
            """
            if len(smoothed) > 3:
                print(f"🔧 Additional axis-aligned projection...")
                extended2 = [smoothed[0]]
                
                for i in range(1, len(smoothed) - 1):
                    curr_x, curr_y, curr_theta = smoothed[i]
                    next_x, next_y, next_theta = smoothed[i + 1]
                    prev_x, prev_y, prev_theta = extended2[-1]
                    
                    # Try to project current waypoint onto the same axis as next waypoint
                    # Option 1: Same X (vertical line)
                    # Option 2: Same Y (horizontal line)
                    
                    best_x, best_y = curr_x, curr_y
                    max_extension = 0
                    
                    # Try projecting to same X as next waypoint (move horizontally)
                    if abs(next_x - curr_x) > 25.0:  # Only if there's significant horizontal distance
                        test_x = next_x
                        test_y = curr_y
                        
                        # Check if we can move from prev to (test_x, test_y)
                        dist1 = math.sqrt((test_x - prev_x)**2 + (test_y - prev_y)**2)
                        angle1 = math.atan2(test_y - prev_y, test_x - prev_x)
                        num_checks1 = max(int(dist1 / 5.0), 20)
                        
                        path1_clear = True
                        for t in range(num_checks1 + 1):
                            ratio = t / num_checks1
                            check_x = prev_x + ratio * (test_x - prev_x)
                            check_y = prev_y + ratio * (test_y - prev_y)
                            if pathfinder.is_collision(check_x, check_y, angle1, grid_resolution):
                                path1_clear = False
                                break
                        
                        # Check if we can move from (test_x, test_y) to next
                        dist2 = math.sqrt((next_x - test_x)**2 + (next_y - test_y)**2)
                        angle2 = math.atan2(next_y - test_y, next_x - test_x)
                        num_checks2 = max(int(dist2 / 5.0), 20)
                        
                        path2_clear = True
                        for t in range(num_checks2 + 1):
                            ratio = t / num_checks2
                            check_x = test_x + ratio * (next_x - test_x)
                            check_y = test_y + ratio * (next_y - test_y)
                            if pathfinder.is_collision(check_x, check_y, angle2, grid_resolution):
                                path2_clear = False
                                break
                        
                        if path1_clear and path2_clear:
                            extension = abs(test_x - curr_x)
                            if extension > max_extension:
                                best_x, best_y = test_x, test_y
                                max_extension = extension
                    
                    # Try projecting to same Y as next waypoint (move vertically)
                    if abs(next_y - curr_y) > 25.0:  # Only if there's significant vertical distance
                        test_x = curr_x
                        test_y = next_y
                        
                        # Check if we can move from prev to (test_x, test_y)
                        dist1 = math.sqrt((test_x - prev_x)**2 + (test_y - prev_y)**2)
                        angle1 = math.atan2(test_y - prev_y, test_x - prev_x)
                        num_checks1 = max(int(dist1 / 5.0), 20)
                        
                        path1_clear = True
                        for t in range(num_checks1 + 1):
                            ratio = t / num_checks1
                            check_x = prev_x + ratio * (test_x - prev_x)
                            check_y = prev_y + ratio * (test_y - prev_y)
                            if pathfinder.is_collision(check_x, check_y, angle1, grid_resolution):
                                path1_clear = False
                                break
                        
                        # Check if we can move from (test_x, test_y) to next
                        dist2 = math.sqrt((next_x - test_x)**2 + (next_y - test_y)**2)
                        angle2 = math.atan2(next_y - test_y, next_x - test_x)
                        num_checks2 = max(int(dist2 / 5.0), 20)
                        
                        path2_clear = True
                        for t in range(num_checks2 + 1):
                            ratio = t / num_checks2
                            check_x = test_x + ratio * (next_x - test_x)
                            check_y = test_y + ratio * (next_y - test_y)
                            if pathfinder.is_collision(check_x, check_y, angle2, grid_resolution):
                                path2_clear = False
                                break
                        
                        if path1_clear and path2_clear:
                            extension = abs(test_y - curr_y)
                            if extension > max_extension:
                                best_x, best_y = test_x, test_y
                                max_extension = extension
                    
                    if max_extension > 25.0:
                        print(f"  Projected waypoint {i+1} by {max_extension:.0f}mm (now axis-aligned with next)")
                        extended2.append((best_x, best_y, curr_theta))
                    else:
                        extended2.append((curr_x, curr_y, curr_theta))
                
                # Always keep last waypoint
                extended2.append(smoothed[-1])
                
                if len(extended2) < len(smoothed):
                    print(f"🚀 Additional axis-aligned: {len(extended2)} waypoints")
                    smoothed = extended2
            """
            
            # FINAL VALIDATION: Check each waypoint can safely rotate to travel toward next waypoint
            print(f"🔒 Final safety check: validating travel rotations...")
            final_path = []
            for i in range(len(smoothed)):
                x, y, theta = smoothed[i]
                
                if i < len(smoothed) - 1:
                    # Calculate travel angle toward next waypoint
                    next_x, next_y, _ = smoothed[i+1]
                    travel_angle = math.atan2(next_y - y, next_x - x)
                    
                    # Check if robot can safely rotate from theta to travel_angle
                    angle_diff = travel_angle - theta
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                    
                    # Check intermediate angles during rotation
                    num_checks = max(18, int(abs(angle_diff) / math.radians(5)))
                    collision_found = False
                    collision_angle = None
                    for j in range(num_checks + 1):
                        ratio = j / num_checks
                        check_angle = theta + ratio * angle_diff
                        if pathfinder.is_collision(x, y, check_angle, grid_resolution):
                            collision_found = True
                            collision_angle = check_angle
                            break
                    
                    if collision_found:
                        print(f"  ⚠ Waypoint {i+1} at ({x:.1f},{y:.1f}): rotation {math.degrees(theta):.1f}°→{math.degrees(travel_angle):.1f}° hits obstacle at {math.degrees(collision_angle):.1f}°")
                        # Try to find a safe orientation
                        safe_angle = None
                        for test_deg in range(-180, 180, 5):
                            test_angle = math.radians(test_deg)
                            if pathfinder.is_collision(x, y, test_angle, grid_resolution):
                                continue
                            # Check if we can rotate to travel angle from this angle
                            test_diff = travel_angle - test_angle
                            while test_diff > math.pi:
                                test_diff -= 2 * math.pi
                            while test_diff < -math.pi:
                                test_diff += 2 * math.pi
                            
                            test_safe = True
                            test_checks = max(18, int(abs(test_diff) / math.radians(5)))
                            for k in range(test_checks + 1):
                                ratio = k / test_checks
                                check_angle = test_angle + ratio * test_diff
                                if pathfinder.is_collision(x, y, check_angle, grid_resolution):
                                    test_safe = False
                                    break
                            
                            if test_safe:
                                safe_angle = test_angle
                                break
                        
                        if safe_angle is not None:
                            print(f"    → Fixed: changed orientation to {math.degrees(safe_angle):.1f}°")
                            final_path.append((x, y, safe_angle))
                        else:
                            # No safe rotation found - need to find a safe intermediate position
                            print(f"    → Searching for safe intermediate waypoint...")
                            
                            # Try multiple distances between current and next waypoint
                            found_safe = False
                            for dist_ratio in [0.2, 0.4, 0.6, 0.8]:
                                mid_x = x + (next_x - x) * dist_ratio
                                mid_y = y + (next_y - y) * dist_ratio
                                mid_to_next_angle = math.atan2(next_y - mid_y, next_x - mid_x)
                                
                                # Check if midpoint position is safe at the required angle
                                if not pathfinder.is_collision(mid_x, mid_y, mid_to_next_angle, grid_resolution):
                                    # Also check if we can get from current to midpoint
                                    curr_to_mid_angle = math.atan2(mid_y - y, mid_x - x)
                                    if not pathfinder.is_collision(x, y, curr_to_mid_angle, grid_resolution):
                                        print(f"      → Found safe position at {dist_ratio*100:.0f}% distance")
                                        final_path.append((x, y, curr_to_mid_angle))
                                        final_path.append((mid_x, mid_y, mid_to_next_angle))
                                        found_safe = True
                                        break
                            
                            if not found_safe:
                                # Check if we can go directly from previous waypoint to next waypoint
                                if len(final_path) > 0:
                                    prev_x, prev_y, prev_theta = final_path[-1]
                                    direct_angle = math.atan2(next_y - prev_x, next_x - prev_x)
                                    direct_dist = math.sqrt((next_x - prev_x)**2 + (next_y - prev_y)**2)
                                    
                                    # Check if direct path is collision-free
                                    num_checks = max(20, int(direct_dist / 50))
                                    direct_safe = True
                                    for j in range(num_checks + 1):
                                        ratio = j / num_checks
                                        check_x = prev_x + ratio * (next_x - prev_x)
                                        check_y = prev_y + ratio * (next_y - prev_y)
                                        if pathfinder.is_collision(check_x, check_y, direct_angle, grid_resolution):
                                            direct_safe = False
                                            break
                                    
                                    if direct_safe:
                                        print(f"    → Skipping waypoint {i+1} - direct path to {i+2} is safe")
                                        # Don't add this waypoint
                                    else:
                                        print(f"    ⚠ Cannot skip - direct path would collide. Keeping waypoint with original orientation")
                                        final_path.append((x, y, theta))
                                else:
                                    print(f"    ⚠ Keeping waypoint with original orientation")
                                    final_path.append((x, y, theta))
                    else:
                        final_path.append((x, y, theta))
                else:
                    # Last waypoint
                    final_path.append((x, y, theta))
            
            smoothed = final_path
            
            # Convert to JSON format
            # Negate theta to match convention: left turns positive, right turns negative
            path_json = []
            for i, (x, y, theta) in enumerate(smoothed):
                path_json.append({
                    "x": round(x, 2),
                    "y": round(y, 2),
                    "theta": round(-math.degrees(theta), 2)
                })
                
                # Print first and last waypoints
                if i < 3 or i >= len(smoothed) - 2:
                    print(f"   {i+1}. ({x:6.1f}, {y:6.1f}) θ={math.degrees(theta):5.1f}°")
            
            if len(smoothed) > 5:
                print(f"   ... ({len(smoothed)-5} waypoints omitted)")
            
            # Calculate path length
            total_dist = 0
            for i in range(len(smoothed) - 1):
                x1, y1, _ = smoothed[i]
                x2, y2, _ = smoothed[i + 1]
                total_dist += math.sqrt((x2-x1)**2 + (y2-y1)**2)
            
            print(f"\n✅ Path generated successfully!")
            print(f"  Waypoints: {len(smoothed)}")
            print(f"  Total distance: {total_dist:.1f}mm ({total_dist/1000:.2f}m)")
            
            # Save to JSON
            data['path'] = path_json
            with open(json_file_path, 'w') as f:
                json.dump(data, f, indent=2)
            
            print(f"\n💾 Saved path to: {json_file_path}")
            print(f"  File contains: {', '.join(data.keys())}")
            
            return True
        else:
            print("\n❌ No path found!")
            print("Possible issues:")
            print("  - Start or goal in collision")
            print("  - Path blocked by obstacles")
            print("  - Try adjusting start/goal positions")
            return False
            
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    if len(sys.argv) < 2:
        print("Usage: python generate_path_hybrid.py <map_json_file>")
        print("Example: python generate_path_hybrid.py ../python_pygame/json/complete_map.json")
        sys.exit(1)
    
    json_file = sys.argv[1]
    
    # Handle relative paths
    if not json_file.startswith('/'):
        json_file = os.path.join(os.getcwd(), json_file)
    
    if not os.path.exists(json_file):
        print(f"✗ File not found: {json_file}")
        sys.exit(1)
    
    print("="*60)
    print("   EUROBOT Hybrid A* Path Generator")
    print("="*60)
    
    success = generate_hybrid_path(json_file)
    print("\n" + "="*60)
    if success:
        print("✅ Path generation completed!")
    else:
        print("❌ Path generation failed!")
    print("="*60)
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
