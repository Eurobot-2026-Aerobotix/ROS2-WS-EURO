#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32


class PneumaticController(Node):
    """
    ROS2 Node for controlling pneumatic systems
    Provides high-level functions for pneumatic control
    """
    
    def __init__(self, node_name='pneumatic_controller'):
        super().__init__(node_name)
        
        # Create QoS profile with reliable settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers for pneumatic systems with reliable QoS
        self.front_pub = self.create_publisher(Int32, '/front/pneumatic', qos_profile)
        self.backward_pub = self.create_publisher(Int32, '/back/pneumatic', qos_profile)

    def front(self, value):
        """
        Publish Int32 value to /front/pneumatic topic
        
        Args:
            value (int): The value to publish
        """
        msg = Int32()
        msg.data = value
        self.front_pub.publish(msg)

    def back(self, value):
        """
        Publish Int32 value to /backward/pneumatic topic
        
        Args:
            value (int): The value to publish
        """
        msg = Int32()
        msg.data = value
        self.backward_pub.publish(msg)


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
