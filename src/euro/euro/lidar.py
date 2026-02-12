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
        self.enemy_radius = 0.35  # meters
        self.robot_width = 0.30  # meters (30cm)
        self.robot_length = 0.30  # meters (30cm)
        # Calculate robot half-diagonal for collision detection
        self.robot_half_diagonal = math.sqrt(self.robot_width**2 + self.robot_length**2) / 2
        self.enemy_x = None
        self.enemy_y = None
        self._recheck_timer = None
        
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
    
    def set_goto_target(self, x, y):
        """
        Set the target coordinates for go_to command
        
        Args:
            x (float): Target x coordinate
            y (float): Target y coordinate
        """
        self.target_x = x
        self.target_y = y
        self.get_logger().info(f"Go-to target set: ({x:.3f}, {y:.3f})")
    
    def clear_goto_target(self):
        """Clear the go_to target when movement is complete"""
        self.target_x = None
        self.target_y = None
        self.get_logger().info("Go-to target cleared")
    
    def check_enemy_collision_with_path(self):
        """Check if enemy circle collides with path from robot to target
        
        Takes into account the robot's 30x30cm rectangular body by using
        the robot's half-diagonal added to the enemy radius.
        """
        if self.enemy_x is None or self.enemy_y is None:
            return False  # No enemy data
        
        if self.target_x is None or self.target_y is None:
            return False  # No target
        
        # Line segment from (current_x, current_y) to (target_x, target_y)
        # Circle at (enemy_x, enemy_y) with effective radius = enemy_radius + robot_half_diagonal
        # This accounts for the robot's 30x30cm body
        
        effective_radius = self.enemy_radius + self.robot_half_diagonal
        
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
    
    def emergency_stop_callback(self, msg):
        """Callback function for EmergencyStop custom message"""
        self.travel_distance = msg.distance
        
        # If we have an emergency and a go_to target is set
        self.get_logger().info(f"target x: {self.target_x} target_y: {self.target_y}")
        if msg.emergency_triggered == 1 and self.target_x is not None and self.target_y is not None:
            # Calculate distance to target
            
            distance_to_target = math.sqrt(
                (self.target_x - self.current_x)**2 + 
                (self.target_y - self.current_y)**2
            )
            
            effective_collision_distance = abs(self.travel_distance)
            
            # If target is closer than the obstacle, don't trigger emergency stop
            if distance_to_target < effective_collision_distance:
                self.cleared_emergency_count+=1
                if(self.cleared_emergency_count>5):
                    self.emergency_status=0
                self.get_logger().info(
                    f"✅ Target ({distance_to_target*1000:.0f}mm) is closer than obstacle "
                    f"({effective_collision_distance*1000:.0f}mm) - continuing to target"
                )
            else: 
                self.cleared_emergency_count=0
                # Only set emergency status if round time is >= 3 seconds
                if self.round_time >= 3.0:
                    self.emergency_status = 1
                    self.get_logger().warning(
                        f"🚨 Emergency ! Obstacle ({effective_collision_distance*1000:.0f}mm) "
                        f"is before target ({distance_to_target*1000:.0f}mm) "
                        f"robot position : ({self.current_x:.3f}, {self.current_y:.3f})"
                    )
                else:
                    self.get_logger().info(
                        f"⏱️  Ignoring obstacle - round time ({self.round_time:.1f}s) < 3s"
                    )
    
    def start_waiting(self):
        """Start the timer logic to recheck enemy collision after 1 second"""
        if self._recheck_timer is not None:
            self._recheck_timer.cancel()
            self._recheck_timer = None
        
        self.get_logger().info("⏱️  Starting 1-second wait before recheck")
        self._recheck_timer = self.create_timer(1.0, self._recheck_enemy_collision)
    
    def _recheck_enemy_collision(self):
        global was_in_emergency
        """Recheck after 1 second if enemy still collides with path to target"""
        if self._recheck_timer is not None:
            self._recheck_timer.cancel()
            self._recheck_timer = None
        
        # Check if enemy circle still intersects with path
        if self.check_enemy_collision_with_path():
            self.emergency_status = 1
            effective_radius = self.enemy_radius + self.robot_half_diagonal
            self.get_logger().warning(
                f"🚨 Recheck: Enemy still blocks path! "
                f"Enemy at ({self.enemy_x:.3f}, {self.enemy_y:.3f}), "
                f"effective collision radius={effective_radius*1000:.0f}mm "
                f"(enemy {self.enemy_radius*1000:.0f}mm + robot {self.robot_half_diagonal*1000:.0f}mm)"
            )
        else:
            self.emergency_status = 0
            self.get_logger().info(
                f"✅ Recheck: Path is clear! Enemy does not block path to target"
            )
    
    def getstatus(self):
        """
        Returns the current emergency status
        
        Returns:
            int: 0 for normal, 1 for emergency
        """
        return self.emergency_status
    
    def get_travel_distance(self):
        """
        Returns the distance to collision
        
        Returns:
            float: Distance in meters until collision (0.0 if no emergency)
        """
        return self.travel_distance

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