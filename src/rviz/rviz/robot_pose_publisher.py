#!/usr/bin/env python3
"""
Robot Pose Publisher Node
Subscribes to robot_pose_raw topic and publishes TF transforms
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class RobotPosePublisher(Node):
    """
    Node that subscribes to robot_pose_raw (Vector3: x, y, z=theta) and publishes
    the robot's transform from map to base_link
    """
    
    def __init__(self):
        super().__init__('robot_pose_publisher')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to robot_pose_raw topic
        self.pose_sub = self.create_subscription(
            Vector3,
            '/robot_pose_raw',
            self.pose_callback,
            10
        )
        
        # Current robot pose (default to 0,0,0)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Create timer to publish TF at 50Hz even if no new pose
        self.create_timer(0.02, self.publish_transform)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Robot Pose Publisher Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info('Initial pose: (0.0, 0.0, 0.0)')
        self.get_logger().info('Subscribing to: /robot_pose_raw (Vector3: x, y, z=theta)')
        self.get_logger().info('Publishing TF: map -> base_link')
        self.get_logger().info('='*60)
    
    def pose_callback(self, msg: Vector3):
        """Callback for robot_pose_raw topic (Vector3: x, y, z=theta)"""
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.z  # z represents theta
        # Uncomment for debugging:
        # self.get_logger().info(f'Updated pose: ({msg.x:.3f}, {msg.y:.3f}, theta={msg.z:.3f})')
    
    def publish_transform(self):
        """Publish the transform from map to base_link"""
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        # Translation
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.translation.z = 0.0
        
        # Rotation (convert yaw to quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.current_theta / 2.0)
        t.transform.rotation.w = math.cos(self.current_theta / 2.0)
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
