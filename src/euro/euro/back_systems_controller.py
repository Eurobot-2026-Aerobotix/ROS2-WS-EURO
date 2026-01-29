#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32
from std_srvs.srv import Trigger


class BackSystemsController(Node):
    """
    ROS2 Node for controlling ESP32 back systems
    Provides high-level functions for back system control
    """
    
    def __init__(self, node_name='back_systems_controller'):
        super().__init__(node_name)
        
        # Create QoS profile with reliable settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers for back systems with reliable QoS
        self.n20_pub = self.create_publisher(Int32, '/back/n20', qos_profile)
        self.kabech_pub = self.create_publisher(Int32, '/back/kabech', qos_profile)
        self.dawer_pub = self.create_publisher(Int32, '/back/dawer', qos_profile)
        
        # Service client for isIDLE
        self.idle_client = self.create_client(Trigger, '/back/isIDLE')

    def n20(self, value):
        """
        Publish Int32 value to /back/n20 topic
        
        Args:
            value (int): The value to publish
        """
        msg = Int32()
        msg.data = value
        self.n20_pub.publish(msg)

    def kabech(self, value):
        """
        Publish Int32 value to /back/kabech topic
        
        Args:
            value (int): The value to publish
        """
        msg = Int32()
        msg.data = value
        self.kabech_pub.publish(msg)

    def dawer(self, value):
        """
        Publish Int32 value to /back/dawer topic
        
        Args:
            value (int): The value to publish
        """
        msg = Int32()
        msg.data = value
        self.dawer_pub.publish(msg)

    def IDLE(self, timeout=2.0):
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
            
            # Poll the future instead of spin_until_future_complete to avoid executor conflicts
            import time
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout:
                    self.get_logger().error('IDLE service call timeout')
                    return False
                time.sleep(0.01)
            
            response = future.result()
            return response.success
        except Exception as e:
            self.get_logger().error(f'IDLE service error: {str(e)}')
            return False


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
