#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import os
import time
import threading
from typing import List, Dict, Any
from collections import deque
from euro.yaml_loader import load_tasks_from_yaml

# Global state variables
IDLE = True
EMERGENCY_STOP = False

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
        
        self.get_logger().info('Strategy Manager initialized')
    
    def robot_status(self, stm32, lidar):
        """Runs every 100ms to check STM32 movement status and lidar obstacles"""
        global IDLE
        global EMERGENCY_STOP
        
        while rclpy.ok():
            # Check emergency stop from lidar
            EMERGENCY_STOP = lidar.getstatus()
            
            # Check if robot is idle
            if not IDLE:
                success, is_completed, _ = stm32.IDLE()
                if not success:
                    self.get_logger().warning("⚠️ Failed to check robot status")
                else:
                    IDLE = is_completed
            
            # Handle emergency stop
            if EMERGENCY_STOP:
                self.get_logger().warning("🚨 Emergency stop activated!")
                success, response = stm32.stop()
            
            time.sleep(0.1)  # 100ms
    
    def stm32_task(self, stm32):
        """Runs every 100ms to process STM32 queue"""
        global IDLE
        
        while rclpy.ok():
            # Check if there are commands in the queue and robot is idle
            if len(self.queues['stm32']) > 0 and IDLE:
                cmd = self.queues['stm32'][0]  # Peek at first command
                
                command = cmd.get('command')
                params = cmd.get('params', {})
                wait = cmd.get('wait', False)
                
                self.get_logger().info(f"📤 STM32: Executing {command} with {params}")
                
                # TODO: Add execution logic here
                # Example:
                # if command == 'move_forward':
                #     stm32.move_forward(params['distance'], params['speed'])
                
                # Remove command from queue after execution
                self.queues['stm32'].popleft()
                
                if wait:
                    IDLE = False  # Set to not idle, robot_status will check
            
            time.sleep(0.1)  # 100ms
    
    def esp1_task(self, esp1_controller=None):
        """Runs every 100ms to process ESP32_1 queue"""
        while rclpy.ok():
            # Check if there are commands in the queue
            if len(self.queues['esp32_1']) > 0:
                cmd = self.queues['esp32_1'][0]  # Peek at first command
                
                command = cmd.get('command')
                params = cmd.get('params', {})
                wait = cmd.get('wait', False)
                
                self.get_logger().info(f"📤 ESP32_1: Executing {command} with {params}")
                
                # TODO: Add execution logic here
                # Example:
                # if command == 'activate_gripper':
                #     esp1_controller.activate_gripper(params['state'])
                
                # Remove command from queue after execution
                self.queues['esp32_1'].popleft()
            
            time.sleep(0.1)  # 100ms
    
    def esp2_task(self, esp2_controller=None):
        """Runs every 100ms to process ESP32_2 queue"""
        while rclpy.ok():
            # Check if there are commands in the queue
            if len(self.queues['esp32_2']) > 0:
                cmd = self.queues['esp32_2'][0]  # Peek at first command
                
                command = cmd.get('command')
                params = cmd.get('params', {})
                wait = cmd.get('wait', False)
                
                self.get_logger().info(f"📤 ESP32_2: Executing {command} with {params}")
                
                # TODO: Add execution logic here
                # Example:
                # if command == 'deploy_mechanism':
                #     esp2_controller.deploy_mechanism(params['height'])
                
                # Remove command from queue after execution
                self.queues['esp32_2'].popleft()
            
            time.sleep(0.1)  # 100ms
    
    def esp3_task(self, esp3_controller=None):
        """Runs every 100ms to process ESP32_3 queue"""
        while rclpy.ok():
            # Check if there are commands in the queue
            if len(self.queues['esp32_3']) > 0:
                cmd = self.queues['esp32_3'][0]  # Peek at first command
                
                command = cmd.get('command')
                params = cmd.get('params', {})
                wait = cmd.get('wait', False)
                
                self.get_logger().info(f"📤 ESP32_3: Executing {command} with {params}")
                
                # TODO: Add execution logic here
                # Example:
                # if command == 'led_pattern':
                #     esp3_controller.led_pattern(params['pattern'], params['color'])
                
                # Remove command from queue after execution
                self.queues['esp32_3'].popleft()
            
            time.sleep(0.1)  # 100ms
    
    def load_task_to_queues(self, task_name: str):
        """Load a task's commands into device queues"""
        # Find the task
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
            strategy.load_task_to_queues(strategy.tasks[0]['name'])
            strategy.display_queue_contents()
        
    except Exception as e:
        strategy.get_logger().error(f"Failed: {str(e)}")
        return
    
    # Start background threads for each device
    strategy.get_logger().info("Starting background threads...")
    
    status_thread = threading.Thread(
        target=strategy.robot_status,
        args=(stm32, lidar),
        daemon=True
    )
    status_thread.start()
    
    stm32_thread = threading.Thread(
        target=strategy.stm32_task,
        args=(stm32,),
        daemon=True
    )
    stm32_thread.start()
    
    esp1_thread = threading.Thread(
        target=strategy.esp1_task,
        args=(None,),  # TODO: Pass ESP32_1 controller when available
        daemon=True
    )
    esp1_thread.start()
    
    esp2_thread = threading.Thread(
        target=strategy.esp2_task,
        args=(None,),  # TODO: Pass ESP32_2 controller when available
        daemon=True
    )
    esp2_thread.start()
    
    esp3_thread = threading.Thread(
        target=strategy.esp3_task,
        args=(None,),  # TODO: Pass ESP32_3 controller when available
        daemon=True
    )
    esp3_thread.start()
    
    strategy.get_logger().info("✓ All threads started\n")
    
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
