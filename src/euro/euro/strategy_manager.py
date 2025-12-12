#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from typing import List, Dict, Any
from collections import deque
from euro.yaml_loader import load_tasks_from_yaml
import time
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
                    while True:
                        self.get_logger().info("      Waiting for IDLE...")
                        time.sleep(0.5)
                        # TODO: Add your IDLE check here
                        # if self.check_idle(device):
                        #     break
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
    
    strategy = StrategyManager()
    
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
        
        # Load first task into queues as example
        if strategy.tasks:
            strategy.load_task_to_queues(strategy.tasks[0]['name'])
        
    except Exception as e:
        strategy.get_logger().error(f"Failed: {str(e)}")
    
    # Keep node alive
    try:
        rclpy.spin(strategy)
    except KeyboardInterrupt:
        strategy.get_logger().info("Shutting down...")
    finally:
        strategy.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
