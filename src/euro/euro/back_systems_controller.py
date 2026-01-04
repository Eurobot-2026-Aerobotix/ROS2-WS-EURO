#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Trigger


class BackSystemsController(Node):
    """
    ROS2 Node for controlling ESP32 back systems
    Provides high-level functions for back system control
    """
    
    def __init__(self, node_name='back_systems_controller'):
        super().__init__(node_name)
        
        # Publishers for back systems
        self.n20_pub = self.create_publisher(Int32, '/back/n20', 10)
        self.kabech_pub = self.create_publisher(Int32, '/back/kabech', 10)
        self.dawer_pub = self.create_publisher(Int32, '/back/dawer', 10)
        
        # Service client for isIDLE
        self.idle_client = self.create_client(Trigger, '/back/isIDLE')
        
        self.get_logger().info('Back Systems Controller initialized')

    def n20(self, value):
        """
        Publish Int32 value to /back/n20 topic
        
        Args:
            value (int): The value to publish
        """
        msg = Int32()
        msg.data = value
        self.n20_pub.publish(msg)
        self.get_logger().info(f'Published to /back/n20: {value}')

    def kabech(self, value):
        """
        Publish Int32 value to /back/kabech topic
        
        Args:
            value (int): The value to publish
        """
        msg = Int32()
        msg.data = value
        self.kabech_pub.publish(msg)
        self.get_logger().info(f'Published to /back/kabech: {value}')

    def dawer(self, value):
        """
        Publish Int32 value to /back/dawer topic
        
        Args:
            value (int): The value to publish
        """
        msg = Int32()
        msg.data = value
        self.dawer_pub.publish(msg)
        self.get_logger().info(f'Published to /back/dawer: {value}')

    def IDLE(self, timeout=10.0):
        """
        Call the /back/isIDLE service
        
        Args:
            timeout (float): Service call timeout in seconds
            
        Returns:
            tuple: (success: bool, message: str)
        """
        if not self.idle_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('/back/isIDLE service not available')
            return False
        
        request = Trigger.Request()
        
        try:
            future = self.idle_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
                self.get_logger().info(f'IDLE response: success={response.success}, message={response.message}')
                return response.success
            else:
                self.get_logger().error('IDLE service call timeout')
                return False
        except Exception as e:
            self.get_logger().error(f'IDLE service error: {str(e)}')
            return False, f'Service error: {str(e)}'


def main(args=None):
    rclpy.init(args=args)
    controller = BackSystemsController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
