#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from typing import List, Dict, Any
from euro.yaml_loader import load_tasks_from_yaml

class StrategyManager(Node):
    """
    Simple strategy manager that loads and displays tasks from YAML
    """
    
    def __init__(self, node_name='strategy_manager'):
        super().__init__(node_name)
        
        # Task storage - simple list
        self.tasks = []
        
        self.get_logger().info('Strategy Manager initialized')
    
    def print_tasks(self):
        """Print all loaded tasks"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"TASKS LIST ({len(self.tasks)} tasks)")
        self.get_logger().info(f"{'='*60}\n")
        
        for task in self.tasks:
            self.get_logger().info(f"Task: {task['name']}")
            self.get_logger().info(f"  Description: {task['description']}")
            self.get_logger().info(f"  Commands: {len(task['sequence'])}")
            
            for idx, cmd in enumerate(task['sequence'], 1):
                self.get_logger().info(f"    [{idx}] {cmd['device']} -> {cmd['command']} {cmd.get('params', {})}")
            
            self.get_logger().info("")


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
        strategy.get_logger().info(f"✓ Loaded {len(strategy.tasks)} tasks")
        strategy.print_tasks()
    except Exception as e:
        strategy.get_logger().error(f"Failed to load tasks: {str(e)}")
    
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
