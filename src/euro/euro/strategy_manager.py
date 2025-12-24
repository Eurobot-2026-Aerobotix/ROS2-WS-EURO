#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import os
import time
import threading
from typing import List, Dict, Any
from collections import deque
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion,Vector3
from euro.yaml_loader import load_tasks_from_yaml
import math
from euro.pathfinder_controller import PathfinderClient
from euro.strategy_executor import robot_status, stm32_task, esp1_task, esp2_task, esp3_task, IDLE, EMERGENCY_STOP

class StrategyManager(Node):
    """
    Simple strategy manager that loads and displays tasks from YAML
    """
    
    def __init__(self, node_name='strategy_manager'):
        super().__init__(node_name)
        
        # Task storage - simple list
        self.tasks = []
        
        # Device queues
        self.queues = {
            'stm32': deque(),
            'esp32_1': deque(),
            'esp32_2': deque(),
            'esp32_3': deque()
        }
        self.create_subscription(
            Vector3,
            '/robot_pose_raw',
            self.robot_pose_callback,
            10
        )
        self.pathfinder = PathfinderClient()
        self.create_timer(1.0, self.try_path)
        self.goal = (0.2, 2.7, 1.57079632679)
        self.path_called = False
        self.get_logger().info("Subscribed to /robot_pose_raw")
        self.get_logger().info('Strategy Manager initialized')
    def robot_pose_callback(self, msg):
        self.latest_robot_pose = msg
        # self.get_logger().info(f"Robot pose updated: {msg}")
    def try_path(self):
        if not self.path_called and self.latest_robot_pose is not None:
            x_goal, y_goal, theta_goal = self.goal
            self.path(x_goal, y_goal, theta_goal)
            self.path_called = True
    def path(self, x_goal, y_goal, theta_goal):
        print("Path function called")
        """Create start and goal PoseStamped from current pose and goal"""
        # Start pose from latest robot pose
        if self.latest_robot_pose is None:
            self.get_logger().error("No robot pose received yet!")
            return
        start_pose = PoseStamped()
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.header.frame_id = "map"
        start_pose.pose.position.x = self.latest_robot_pose.x
        start_pose.pose.position.y = self.latest_robot_pose.y
        start_pose.pose.position.z = 0.0
        start_pose.pose.orientation = self.yaw_to_quaternion(self.latest_robot_pose.z)

        # Goal pose from arguments
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = x_goal
        goal_pose.pose.position.y = y_goal
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = self.yaw_to_quaternion(theta_goal)
        stm32_commands, success = self.pathfinder.get_path(start_pose, goal_pose)
        last_cmd=None
        for cmd in stm32_commands:
            self.get_logger().info(f"Path command: {cmd}")
            values = cmd.replace('go_to(', '').replace(')', '').split(',')
            x = float(values[0].strip())
            y = float(values[1].strip())
            speed = int(values[2].strip())
            self.get_logger().info(
                f"Parsed → x={x}, y={y}, speed={speed}"
            )
            if(cmd != last_cmd):
                self.queues['stm32'].append({
                    'command': 'go_to',
                    'params': {
                        'x': x,
                        'y': y,
                        'speed': 700.0
                    },
                    'wait': True
                })
            last_cmd=cmd
        if success :
            self.queues['stm32'].append({
                'command': 'orientate',
                'params': {
                    'angle': 180.0,
                    'is_radian': 0.0
                },
            })
        self.get_logger().info(f"STM32 Queue: {self.queues['stm32']}")

    def quaternion_to_yaw(self, quat: Quaternion) -> float:
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat
    
    def load_task_to_queues(self, task_name: str):
        """Load a task's commands into device queues"""
        # Find the task
        global IDLE
        task = None
        for t in self.tasks:
            if t['name'] == task_name:
                task = t
                break
        
        if not task:
            self.get_logger().error(f"Task '{task_name}' not found!")
            return False
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"LOADING TASK: {task['name']}")
        self.get_logger().info(f"Description: {task['description']}")
        self.get_logger().info(f"{'='*60}\n")
        
        # Iterate through sequence and add to queues
        for idx, cmd in enumerate(task['sequence'], 1):
            device = cmd.get('device')
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            self.get_logger().info(f"[{idx}/{len(task['sequence'])}] {device} -> {command}")
            
            # Add command to device queue
            if device in self.queues:
                self.queues[device].append(cmd)
                self.get_logger().info(f"      ✓ Added to {device} queue (wait: {wait})")
            else:
                self.get_logger().warn(f"      ⚠ Unknown device: {device}")
            
            if wait:
                self.get_logger().info(f"      ⏳ Waiting for {device} to complete...")
                # TODO: Replace with actual IDLE check
                try:
                    while not IDLE:
                        self.get_logger().info("      Waiting for IDLE...")
                        time.sleep(0.5)
                except KeyboardInterrupt:
                    self.get_logger().warn("      Interrupted by user!")
                    raise
                self.get_logger().info(f"      ✓ {device} completed!")
                
            
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"QUEUE STATUS:")
        for device_name, queue in self.queues.items():
            self.get_logger().info(f"  {device_name}: {len(queue)} commands")
        self.get_logger().info(f"{'='*60}\n")
        
        return True
    
    def display_queue_contents(self):
        """Display the full contents of all queues with parameters"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"QUEUE CONTENTS (with parameters):")
        self.get_logger().info(f"{'='*60}\n")
        
        for device_name, queue in self.queues.items():
            if len(queue) > 0:
                self.get_logger().info(f"📦 {device_name} ({len(queue)} commands):")
                for idx, cmd in enumerate(queue, 1):
                    self.get_logger().info(f"   [{idx}] Command: {cmd.get('command')}")
                    self.get_logger().info(f"       Params: {cmd.get('params', {})}")
                    self.get_logger().info(f"       Wait: {cmd.get('wait', False)}")
                    if 'delay' in cmd:
                        self.get_logger().info(f"       Delay: {cmd.get('delay')}s")
                self.get_logger().info("")
            else:
                self.get_logger().info(f"📭 {device_name}: empty")
        
        self.get_logger().info(f"{'='*60}\n")


def main(args=None):
    rclpy.init(args=args)
    
    # Import controllers
    from euro.stm32_controller import STM32Controller
    from euro.lidar import Lidar
    from euro.pathfinder_controller import PathfinderClient
    # Initialize controllers
    stm32 = STM32Controller('rpi_stm32')
    lidar = Lidar()
    # Create strategy manager
    strategy = StrategyManager()
    
    # Create executor for ROS2 nodes
    executor = MultiThreadedExecutor()
    executor.add_node(stm32)
    executor.add_node(lidar)
    executor.add_node(strategy)
     # Start background threads for each device
    strategy.get_logger().info("Starting background threads...")
    
    status_thread = threading.Thread(
        target=robot_status,
        args=(strategy, stm32, lidar),
        daemon=True
    )
    status_thread.start()
    
    stm32_thread = threading.Thread(
        target=stm32_task,
        args=(strategy, stm32),
        daemon=True
    )
    stm32_thread.start()
    
    esp1_thread = threading.Thread(
        target=esp1_task,
        args=(strategy, None),  # TODO: Pass ESP32_1 controller when available
        daemon=True
    )
    esp1_thread.start()
    
    esp2_thread = threading.Thread(
        target=esp2_task,
        args=(strategy, None),  # TODO: Pass ESP32_2 controller when available
        daemon=True
    )
    esp2_thread.start()
    
    esp3_thread = threading.Thread(
        target=esp3_task,
        args=(strategy, None),  # TODO: Pass ESP32_3 controller when available
        daemon=True
    )
    esp3_thread.start()
    
    strategy.get_logger().info("✓ All threads started\n")
    # Get tasks file path
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share_dir = get_package_share_directory('euro')
        tasks_file = os.path.join(package_share_dir, 'config', 'tasks.yaml')
    except:
        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        tasks_file = os.path.join(package_dir, 'config', 'tasks.yaml')
    
    # Load tasks
    try:
        strategy.get_logger().info(f"Loading tasks from: {tasks_file}")
        strategy.tasks = load_tasks_from_yaml(tasks_file)
        strategy.get_logger().info(f"✓ Loaded {len(strategy.tasks)} tasks\n")
        
        # Load first task into queues
        if strategy.tasks:
            x_goal = 1760.0
            #strategy.load_task_to_queues(strategy.tasks[0]['name'])
            #strategy.display_queue_contents()
        
    except Exception as e:
        strategy.get_logger().error(f"Failed: {str(e)}")
        return
    
   
    
    # Keep node alive
    try:
        
        executor.spin()
    except KeyboardInterrupt:
        strategy.get_logger().info("Shutting down...")
    finally:
        strategy.destroy_node()
        stm32.destroy_node()
        lidar.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
