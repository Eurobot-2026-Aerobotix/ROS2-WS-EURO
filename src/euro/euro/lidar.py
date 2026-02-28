#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from lidar_obstacle_warning_msgs.msg import EmergencyStop
from geometry_msgs.msg import Vector3, PoseStamped
class Lidar(Node):
    """
    Lidar node that subscribes to emergency topic
    """
    
    def __init__(self, node_name='lidar'):
        super().__init__(node_name)
        
        # Log deduplication - track last N messages to prevent spam
        self._last_log_messages = []
        self._max_log_history = 5
        
        # Emergency status variable (0 = normal, 1 = emergency)
        self.emergency_status = 0
        self.travel_distance = 0.0
        
        # Round time tracking
        self.round_time = 0.0
        
        # Go-to target tracking
        self.target_x = None
        self.target_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.cleared_emergency_count = 0
        # Obstacle parameters
        self.obstacle_radius = 0.25  # meters
        self.enemy_radius = 0.35  # meters - for real-time collision detection
        self.enemy_radius_for_pathfinding = 0.25  # meters - smaller for path generation
        self.robot_width = 0.30  # meters (30cm)
        self.robot_length = 0.30  # meters (30cm)
        # Calculate robot half-diagonal for collision detection
        self.robot_half_diagonal = math.sqrt(self.robot_width**2 + self.robot_length**2) / 2
        self.enemy_x = None
        self.enemy_y = None
        self._recheck_timer = None
        self._recheck_count = 0  # Counter for 0.1s checks
        self.generate_path = 0  # Flag to indicate if new path generation is needed
        self.clear_on=True
        self.path_just_generated = False  # Flag to suppress emergency after new path generated
        # Subscribe to /emergency_stop (custom EmergencyStop message)
        self.emergency_stop_subscription = self.create_subscription(
            EmergencyStop,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )
        
        # Subscribe to robot pose
        self.pose_subscription = self.create_subscription(
            Vector3,
            '/robot_pose_raw',
            self.pose_callback,
            10
        )
        
        # Subscribe to enemy obstacle position
        self.enemy_subscription = self.create_subscription(
            PoseStamped,
            '/obstacle_relative',
            self.enemy_callback,
            10
        )
        
        self.get_logger().info('Lidar node initialized - subscribed to /emergency_stop, /robot_pose_raw, and /obstacle_relative topics')
    
    def log_once(self, message: str, level: str = 'info'):
        """Log a message only if it's different from recent messages (prevents spam)"""
        # Check if this exact message was logged recently
        if message in self._last_log_messages:
            return  # Skip duplicate
        
        # Log the message
        if level == 'info':
            self.get_logger().info(message)
        elif level == 'warn' or level == 'warning':
            self.get_logger().warning(message)
        elif level == 'error':
            self.get_logger().error(message)
        elif level == 'debug':
            self.get_logger().debug(message)
        
        # Update history
        self._last_log_messages.append(message)
        if len(self._last_log_messages) > self._max_log_history:
            self._last_log_messages.pop(0)

    def pose_callback(self, msg):
        """Update current robot position"""
        self.current_x = msg.x
        self.current_y = msg.y
    
    def enemy_callback(self, msg):
        """Update enemy obstacle position"""
        self.enemy_x = msg.pose.position.x
        self.enemy_y = msg.pose.position.y
    
    def set_round_time(self, time: float):
        """
        Set the current round time
        
        Args:
            time (float): Round time in seconds
        """
        self.round_time = time
    
    def set_goto_target(self, x, y, from_pathfinding=False):
        """
        Set the target coordinates for go_to command
        
        Args:
            x (float): Target x coordinate
            y (float): Target y coordinate
            from_pathfinding (bool): True if this target is from a newly generated path
        """
        self.target_x = x
        self.target_y = y
        self.get_logger().info(f"Go-to target set: ({x:.3f}, {y:.3f})")
        # Start checking for enemy collision
        self.clear_on=True
        
        # If this is from pathfinding, use relaxed collision threshold
        if from_pathfinding:
            self.path_just_generated = True
            self.get_logger().info("🛤️  New path generated - using relaxed collision threshold")
        
    
    def clear_goto_target(self):
        """Clear the go_to target when movement is complete"""
        self.target_x = None
        self.target_y = None
        self.path_just_generated = False
        self.get_logger().info("Go-to target cleared")
    
    def check_enemy_collision_with_path(self):
        """Check if enemy circle collides with path from robot to target
        
        Takes into account the robot's 30x30cm rectangular body by using
        the robot's half-diagonal added to the enemy radius.
        Uses the smaller pathfinding radius (0.25m) for consistency with generated paths.
        """
        if self.enemy_x is None or self.enemy_y is None:
            return False  # No enemy data
        
        if self.target_x is None or self.target_y is None:
            return False  # No target
        
        # Line segment from (current_x, current_y) to (target_x, target_y)
        # Circle at (enemy_x, enemy_y) with effective radius = enemy_radius_for_pathfinding + robot_half_diagonal
        # Use the smaller pathfinding radius to match how paths are generated
        
        effective_radius = self.enemy_radius_for_pathfinding + self.robot_half_diagonal
        
        # Vector from robot to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        
        # Vector from robot to enemy
        fx = self.current_x - self.enemy_x
        fy = self.current_y - self.enemy_y
        
        # Quadratic equation coefficients for line-circle intersection
        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = (fx * fx + fy * fy) - effective_radius * effective_radius
        
        discriminant = b * b - 4 * a * c
        
        # No intersection if discriminant is negative
        if discriminant < 0:
            return False
        
        # Calculate intersection points
        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        
        # Check if intersection is within the line segment (0 <= t <= 1)
        if (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1):
            return True
        
        return False
    
    def is_enemy_on_side(self):
        """Check if enemy robot is to the left or right (not directly in path)
        
        Returns:
            bool: True if enemy is on the side (left/right), False if in front/behind
        """
        if self.enemy_x is None or self.enemy_y is None:
            return False
        
        if self.target_x is None or self.target_y is None:
            return False
        
        # Vector from robot to target (direction of travel)
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        
        # Normalize direction vector
        path_length = math.sqrt(dx * dx + dy * dy)
        if path_length < 0.015:  # Very close to target (15mm) - don't care about obstacles
            self.log_once(
                f"🎯 Very close to target ({path_length*1000:.0f}mm) - ignoring side check"
            )
            return True
        
        dx_norm = dx / path_length
        dy_norm = dy / path_length
        
        # Vector from robot to enemy
        enemy_dx = self.enemy_x - self.current_x
        enemy_dy = self.enemy_y - self.current_y
        
        # Distance to enemy
        enemy_distance = math.sqrt(enemy_dx * enemy_dx + enemy_dy * enemy_dy)
        
        # Calculate dot product (forward/backward component)
        # and cross product magnitude (left/right component)
        forward_component = enemy_dx * dx_norm + enemy_dy * dy_norm
        lateral_component = abs(enemy_dx * dy_norm - enemy_dy * dx_norm)
        
        # If enemy is behind us (negative forward component)
        if forward_component < 0:
            self.log_once(
                f"⬅️ Enemy is behind us (forward={forward_component*1000:.0f}mm) - not blocking path"
            )
            return True
        
        # If enemy is mostly to the side (left/right)
        # Improved logic: enemy is on side if lateral displacement is significantly greater than forward
        # Use a threshold: lateral_component > 1.2 * forward_component (more aggressive side detection)
        if lateral_component > 1.2 * abs(forward_component):
            self.log_once(
                f"🔄 Enemy is on the side (lateral={lateral_component*1000:.0f}mm, "
                f"forward={forward_component*1000:.0f}mm, distance={enemy_distance*1000:.0f}mm) - not blocking path"
            )
            return True
        
        # Additional check: if enemy is far to the side (> 400mm laterally), consider it not blocking
        if lateral_component > 0.4:
            self.log_once(
                f"↔️ Enemy is far to the side ({lateral_component*1000:.0f}mm lateral) - not blocking path"
            )
            return True
        
        return False
    
    def emergency_stop_callback(self, msg):
        """Callback function for EmergencyStop custom message"""
        self.travel_distance = msg.distance
        
        # If we have an emergency and a go_to target is set
        if msg.emergency_triggered == 1 and self.target_x is not None and self.target_y is not None:
            # Calculate distance to target
            
            distance_to_target = math.sqrt(
                (self.target_x - self.current_x)**2 + 
                (self.target_y - self.current_y)**2
            )
            
            effective_collision_distance = abs(self.travel_distance)
            
            # If we're very close to target (within 15mm), ignore obstacles - we've basically arrived
            if distance_to_target < 0.015:  # 15mm tolerance
                if self.emergency_status != 1:
                    self.emergency_status = 0
                self.log_once(
                    f"✅ At target location ({distance_to_target*1000:.0f}mm away) - ignoring obstacles"
                )
                return
            
            # Check if enemy is on the side (left/right) or behind - not my collision
            if self.is_enemy_on_side():
                self.emergency_status = 0
                self.log_once(
                    "✅ Enemy is on the side - not causing collision, continuing"
                )
                return
            
            # If target is closer than or equal to the obstacle, don't trigger emergency stop
            if distance_to_target <= effective_collision_distance:
                if self.emergency_status != 1:
                    self.emergency_status=0
                self.log_once(
                    f"✅ Target ({distance_to_target*1000:.0f}mm) is closer than obstacle "
                    f"({effective_collision_distance*1000:.0f}mm) - continuing to target"
                )
            else: 
                self.cleared_emergency_count=0
                # Only set emergency status if round time is >= 1 second
                if self.round_time >= 1.0:
                    # When following a generated path, use reduced collision margin
                    # Path was generated with 0.25m enemy radius, so accept being in detection zone
                    if self.path_just_generated:
                        # Only stop if we're ACTUALLY colliding (distance to collision ≈ 0)
                        # Normal detection triggers at 350mm, but generated path stays >250mm away
                        if effective_collision_distance > 0.05:  # 50mm safety margin
                            self.log_once(
                                f"🛤️  Following generated path - distance to collision: "
                                f"{effective_collision_distance*1000:.0f}mm (safe, >50mm)"
                            )
                            self.clear_on=True
                            self.emergency_status = 0
                        else:
                            # Only stop if actual collision imminent (<50mm to impact)
                            self.emergency_status = 1
                            self.clear_on=False
                            self.get_logger().warning(
                               f"⚠️  Path execution - ACTUAL collision imminent! "
                               f"Distance: {effective_collision_distance*1000:.0f}mm < 50mm"
                            )
                    else:
                        self.emergency_status = 1
                        self.clear_on=False
                        # Start checking enemy collision for 1 second
                        self.get_logger().warning(
                            f"🚨 Emergency ! Obstacle ({effective_collision_distance*1000:.0f}mm) "
                            f"is before target ({distance_to_target*1000:.0f}mm) "
                            f"robot position : ({self.current_x:.3f}, {self.current_y:.3f})"
                        )
                else:
                    self.log_once(
                        f"⏱️  Ignoring obstacle - round time ({self.round_time:.1f}s) < 3s"
                    )
        else: 
            if self.clear_on:
                self.emergency_status=0

    def start_enemy_collision_check(self):
        """Start checking enemy collision every 0.1s for up to 1 second"""
        if self._recheck_timer is not None:
            self._recheck_timer.cancel()
            self._recheck_timer = None
        
        # Reset counter and generate_path flag
        self._recheck_count = 0
        self.generate_path = 0
        
        self.get_logger().info("⏱️  Starting enemy collision checks every 0.1s for 1 second")
        self._recheck_timer = self.create_timer(0.1, self._recheck_enemy_collision)
    
    def _recheck_enemy_collision(self):
        """Check every 0.1s if enemy collides with path. Stop early if clear, or after 1s set generate_path"""
        self._recheck_count += 1
        
        # First check if enemy is on the side (not blocking)
        if self.is_enemy_on_side():
            if self._recheck_timer is not None:
                self._recheck_timer.cancel()
                self._recheck_timer = None
            self.emergency_status = 0
            self.generate_path = 0
            self._recheck_count = 0
            self.log_once(
                f"✅ Enemy is on the side during recheck - not blocking, clearing emergency"
            )
            return
        
        # Check if enemy circle still intersects with path
        collision_detected = self.check_enemy_collision_with_path()
        
        # If no collision detected, stop checking and clear emergency
        if not collision_detected:
            if self._recheck_timer is not None:
                self._recheck_timer.cancel()
                self._recheck_timer = None
            self.emergency_status = 0
            self.generate_path = 0
            self._recheck_count = 0
            self.log_once(
                f"✅ Enemy collision cleared! Path is clear, emergency_status=0, generate_path=0"
            )
            return
        
        # After 10 checks (1 second total), set generate_path flag
        if self._recheck_count >= 10:
            if self._recheck_timer is not None:
                self._recheck_timer.cancel()
                self._recheck_timer = None
            
            self.emergency_status = 1
            self.generate_path = 1
            effective_radius = self.enemy_radius_for_pathfinding + self.robot_half_diagonal
            self._recheck_count = 0
            self.get_logger().warning(
                f"🚨 After 1s: Enemy still blocks path! emergency_status=1, generate_path=1 "
                f"Enemy at ({self.enemy_x:.3f}, {self.enemy_y:.3f}), "
                f"effective collision radius={effective_radius*1000:.0f}mm "
                f"(enemy {self.enemy_radius_for_pathfinding*1000:.0f}mm + robot {self.robot_half_diagonal*1000:.0f}mm)"
            )
    
    def getstatus(self):
        """
        Returns the current emergency status
        
        Returns:
            int: 0 for normal, 1 for emergency
        """
        if self.target_x is not None:
            return self.emergency_status
        else:
            return 0  # No target means no emergency
    
    def get_travel_distance(self):
        """
        Returns the distance to collision
        
        Returns:
            float: Distance in meters until collision (0.0 if no emergency)
        """
        return self.travel_distance
    
    def get_path_status(self):
        """Returns whether new path generation is needed"""
        return self.generate_path
    
    def get_enemy_radius_for_pathfinding(self):
        """Returns the smaller enemy radius to use for path generation (0.25m)"""
        return self.enemy_radius_for_pathfinding

def main(args=None):
    rclpy.init(args=args)
    lidar = Lidar('lidar')
    
    try:
        rclpy.spin(lidar)
    except KeyboardInterrupt:
        lidar.get_logger().info('Lidar node shutting down...')
    finally:
        lidar.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()