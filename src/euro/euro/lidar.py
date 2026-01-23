#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Lidar(Node):
    """
    Lidar node that subscribes to emergency topic
    """
    
    def __init__(self, node_name='lidar'):
        super().__init__(node_name)
        
        # Emergency status variable (0 = normal, 1 = emergency)
        self.emergency_status = 0
        
        # Subscribe to /emergency topic
        self.emergency_subscription = self.create_subscription(
            Int32,
            '/emergency_break',
            self.emergency_callback,
            10
        )
        self.get_logger().info('Lidar node initialized - subscribed to /emergency_break topic')

    def emergency_callback(self, msg):
        """Callback function for emergency topic"""
        self.emergency_status = msg.data
        if self.emergency_status == 1:
            self.get_logger().warning("🚨 Emergency signal received!")
    
    def getstatus(self):
        """
        Returns the current emergency status
        
        Returns:
            int: 0 for normal, 1 for emergency
        """
        return self.emergency_status

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