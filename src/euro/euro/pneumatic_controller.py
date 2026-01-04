#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class PneumaticController(Node):
    """
    ROS2 Node for controlling pneumatic systems
    Provides high-level functions for pneumatic control
    """
    
    def __init__(self, node_name='pneumatic_controller'):
        super().__init__(node_name)
        
        # Publishers for pneumatic systems
        self.front_pub = self.create_publisher(Int32, '/front/pneumatic', 10)
        self.backward_pub = self.create_publisher(Int32, '/backward/pneumatic', 10)
        
        self.get_logger().info('Pneumatic Controller initialized')

    def front(self, value):
        """
        Publish Int32 value to /front/pneumatic topic
        
        Args:
            value (int): The value to publish
        """
        msg = Int32()
        msg.data = value
        self.front_pub.publish(msg)
        self.get_logger().info(f'Published to /front/pneumatic: {value}')

    def back(self, value):
        """
        Publish Int32 value to /backward/pneumatic topic
        
        Args:
            value (int): The value to publish
        """
        msg = Int32()
        msg.data = value
        self.backward_pub.publish(msg)
        self.get_logger().info(f'Published to /backward/pneumatic: {value}')


def main(args=None):
    rclpy.init(args=args)
    controller = PneumaticController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
