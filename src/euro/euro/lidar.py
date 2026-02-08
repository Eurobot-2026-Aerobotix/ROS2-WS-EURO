#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from lidar_obstacle_warning_msgs.msg import EmergencyStop

class Lidar(Node):
    """
    Lidar node that subscribes to emergency topic
    """
    
    def __init__(self, node_name='lidar'):
        super().__init__(node_name)
        
        # Emergency status variable (0 = normal, 1 = emergency)
        self.emergency_status = 0
        self.travel_distance = 0.0
        
        # Subscribe to /emergency topic (legacy Int32)
        self.emergency_subscription = self.create_subscription(
            Int32,
            '/emergency_break',
            self.emergency_callback,
            10
        )
        
        # Subscribe to /emergency_stop (custom EmergencyStop message)
        self.emergency_stop_subscription = self.create_subscription(
            EmergencyStop,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )
        self.get_logger().info('Lidar node initialized - subscribed to /emergency_break and /emergency_stop topics')

    def emergency_callback(self, msg):
        """Callback function for emergency topic (legacy Int32)"""
        self.emergency_status = msg.data
        if self.emergency_status == 1:
            self.get_logger().warning("🚨 Emergency signal received!")
        else:
            self.get_logger().info("✅ Emergency signal cleared")
    
    def emergency_stop_callback(self, msg):
        """Callback function for EmergencyStop custom message"""
        self.emergency_status = msg.emergency_triggered
        self.travel_distance = msg.distance
        
        if self.emergency_status == 1:
            self.get_logger().warning(
                f"🚨 Emergency Stop! Travel distance: {self.travel_distance*1000:.1f}mm ({self.travel_distance:.3f}m)"
            )
        else:
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