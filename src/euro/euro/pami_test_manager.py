#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import os
import sys
import time
import yaml
from pathlib import Path
from euro.pami_controller import PamiController


class PamiTestManager(Node):
    """
    Test manager that loads and controls PAMI strategies for testing
    Similar to strategy_manager but focused only on PAMI operations
    """
    
    def __init__(self, node_name='pami_test_manager'):
        super().__init__(node_name)
        
        # Strategy name from YAML
        self.strategy_name = None
        
        # PAMI controller instance
        self.pami_controller = None
        
        self.get_logger().info('PAMI Test Manager initialized')
    
    def load_strategy_info(self, strategy_file: str):
        """
        Load strategy information from YAML file
        
        Args:
            strategy_file: Path to strategy YAML file
        """
        if not os.path.exists(strategy_file):
            self.get_logger().error(f"Strategy file not found: {strategy_file}")
            return False
        
        try:
            with open(strategy_file, 'r') as file:
                data = yaml.safe_load(file)
                
                # Get strategy name (try both 'name' and 'strategy_name' fields)
                self.strategy_name = data.get('name') or data.get('strategy_name')
                
                if not self.strategy_name:
                    self.get_logger().error("No 'name' or 'strategy_name' found in YAML file")
                    return False
                
                self.get_logger().info(f"✓ Loaded strategy: {self.strategy_name}")
                return True
                
        except Exception as e:
            self.get_logger().error(f"Error loading strategy file: {e}")
            return False
    
    def load_pami_strategies(self, strategy_file: str):
        """
        Load PAMI strategies based on the strategy file path
        
        Args:
            strategy_file: Path to strategy YAML file
        """
        # Determine pami_strategies folder path
        pami_folder = os.path.join(os.path.dirname(strategy_file), '..', 'pami_strategies')
        pami_folder = os.path.abspath(pami_folder)
        
        if not os.path.exists(pami_folder):
            self.get_logger().warn(f"PAMI strategies folder not found: {pami_folder}")
            return False
        
        self.get_logger().info(f"Loading PAMI strategies from: {pami_folder}")
        
        # Create PAMI controller
        self.pami_controller = PamiController(
            self.strategy_name, 
            pami_folder, 
            'pami_controller_test'
        )
        
        return True
    
    def start_all_pamis(self):
        """
        Send start signal to all PAMI robots
        """
        if self.pami_controller:
            self.get_logger().info("🚀 Starting all PAMIs...")
            self.pami_controller.start_pamis()
        else:
            self.get_logger().warn("No PAMI controller loaded")
    
    def start_pami(self, pami_number: int):
        """
        Start a specific PAMI robot
        
        Args:
            pami_number: PAMI robot number (1-7)
        """
        if self.pami_controller:
            self.get_logger().info(f"🚀 Starting PAMI {pami_number}...")
            self.pami_controller.start_pami(pami_number)
        else:
            self.get_logger().warn("No PAMI controller loaded")


def main(args=None):
    """
    Main function to run PAMI test manager
    
    Usage:
        ros2 run euro pami_test_manager <strategy_file_path>
    
    Example:
        ros2 run euro pami_test_manager /path/to/strategies/7etem.yaml
    """
    rclpy.init(args=args)
    
    # Create PAMI test manager
    manager = PamiTestManager()
    
    # Create executor for ROS2 node
    executor = MultiThreadedExecutor()
    executor.add_node(manager)
    
    # Get strategy file path from command line arguments
    if len(sys.argv) > 1:
        strategy_file = sys.argv[1]
    else:
        manager.get_logger().error("No strategy file path provided!")
        manager.get_logger().info("Usage: ros2 run euro pami_test_manager <strategy_file_path>")
        manager.get_logger().info("Example: ros2 run euro pami_test_manager ~/EUROBOT/Eurobot_Dashboard/strategies/7etem.yaml")
        rclpy.shutdown()
        return
    
    # Load strategy information
    if not manager.load_strategy_info(strategy_file):
        manager.get_logger().error("Failed to load strategy information")
        rclpy.shutdown()
        return
    
    # Load PAMI strategies
    if not manager.load_pami_strategies(strategy_file):
        manager.get_logger().error("Failed to load PAMI strategies")
        rclpy.shutdown()
        return
    
    manager.get_logger().info("\n" + "="*60)
    manager.get_logger().info("PAMI Test Manager Ready!")
    manager.get_logger().info(f"Strategy: {manager.strategy_name}")
    manager.get_logger().info("="*60 + "\n")
    
    # Wait a moment for topics to be set up
    time.sleep(1.0)
    
    # Option to auto-start PAMIs (you can modify this behavior)
    manager.get_logger().info("Waiting 3 seconds before starting PAMIs...")
    manager.get_logger().info("(Press Ctrl+C to cancel)")
    
    try:
        time.sleep(3.0)
        
        # Start all PAMIs
        manager.start_all_pamis()
        
        manager.get_logger().info("\n✅ All PAMIs started!")
        manager.get_logger().info("Test manager will keep running. Press Ctrl+C to stop.\n")
        
        # Keep spinning to maintain publishers
        rclpy.spin(manager)
        
    except KeyboardInterrupt:
        manager.get_logger().info("\n🛑 Interrupted by user")
    
    # Cleanup
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
