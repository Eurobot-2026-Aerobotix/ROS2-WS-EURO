#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from lidar_obstacle_warning_msgs.msg import EmergencyStop
from geometry_msgs.msg import Vector3

class Lidar(Node):
    """
    Lidar node that subscribes to emergency topic
    """
    
    def __init__(self, node_name='lidar'):
        super().__init__(node_name)
        
        # Emergency status variable (0 = normal, 1 = emergency)
        self.emergency_status = 0
        self.travel_distance = 0.0
        
        # Go-to target tracking
        self.target_x = None
        self.target_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.cleared_emergency_count = 0
        # Obstacle parameters
        self.obstacle_radius = 0.25  # meters
        
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
        
        self.get_logger().info('Lidar node initialized - subscribed to /emergency_stop and /robot_pose_raw topics')

    def pose_callback(self, msg):
        """Update current robot position"""
        self.current_x = msg.x
        self.current_y = msg.y
    
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
                self.emergency_status = 1
                self.get_logger().warning(
                    f"🚨 Emergency Stop! Obstacle ({effective_collision_distance*1000:.0f}mm) "
                    f"is before target ({distance_to_target*1000:.0f}mm)"
                    f"robot position : ({self.current_x:.3f}, {self.current_y:.3f})"
                )
        elif msg.emergency_triggered == 1:
            self.cleared_emergency_count=0
            # No target set, trigger emergency normally
            self.emergency_status = 1
            self.get_logger().warning(
                    f"🚨 Emergency Stop! "
                    f"robot position : ({self.current_x:.3f}, {self.current_y:.3f})")
        else:
            # Emergency cleared
            self.cleared_emergency_count += 1
            if self.cleared_emergency_count > 5:
                self.emergency_status = 0
                self.get_logger().info("✅ Emergency cleared")
    
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