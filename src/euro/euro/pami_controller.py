#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import yaml
import os
from pathlib import Path
from typing import List, Dict, Any


class PamiController(Node):
    """
    Controller that loads PAMI strategies and publishes them to appropriate topics
    """
    
    def __init__(self, strategy_name: str, pami_folder_path: str, node_name='pami_controller'):
        super().__init__(node_name)
        
        self.strategy_name = strategy_name
        self.pami_folder_path = pami_folder_path
        
        # Dictionary to store publishers for each pami
        self.pami_publishers = {}
        
        # Dictionary to store start publishers for each pami
        self.pami_start_publishers = {}
        
        # Load and publish PAMI strategies
        self.load_and_publish_pami_strategies()
        
        self.get_logger().info(f'PAMI Controller initialized for strategy: {strategy_name}')
    
    def load_pami_yaml(self, yaml_file: str) -> Dict[str, Any]:
        """
        Load a single PAMI YAML file
        
        Args:
            yaml_file: Path to YAML file
            
        Returns:
            Dictionary with strategy data or None if file doesn't exist
        """
        if not os.path.exists(yaml_file):
            return None
        
        try:
            with open(yaml_file, 'r') as file:
                data = yaml.safe_load(file)
                return data
        except Exception as e:
            self.get_logger().error(f"Error loading {yaml_file}: {e}")
            return None
    
    def sequence_to_float_array(self, sequence: List[List]) -> List[float]:
        """
        Convert sequence list to flat float array with padding
        Each command is padded to exactly 4 floats
        
        Args:
            sequence: List of lists containing command data
            
        Returns:
            Flattened list of floats (each command = 4 floats)
        """
        flat_array = []
        for cmd in sequence:
            # Convert command values to floats
            cmd_floats = []
            for value in cmd:
                try:
                    cmd_floats.append(float(value))
                except (ValueError, TypeError):
                    self.get_logger().warn(f"Could not convert '{value}' to float, using 0.0")
                    cmd_floats.append(0.0)
            
            # Pad to exactly 4 floats if needed
            while len(cmd_floats) < 4:
                cmd_floats.append(0.0)
            
            # Add the 4 floats to the array
            flat_array.extend(cmd_floats[:4])  # Take only first 4 values if more than 4
        
        return flat_array
    
    def load_and_publish_pami_strategies(self):
        """
        Load all PAMI YAML files from the folder and publish matching strategies
        """
        if not os.path.exists(self.pami_folder_path):
            self.get_logger().error(f"PAMI folder not found: {self.pami_folder_path}")
            return
        
        self.get_logger().info(f"Loading PAMI strategies from: {self.pami_folder_path}")
        self.get_logger().info(f"Filtering by strategy name: {self.strategy_name}")
        
        # Get all YAML files in the pami_strategies folder
        pami_files = list(Path(self.pami_folder_path).glob('*.yaml'))
        self.get_logger().info(f"Found {len(pami_files)} YAML files")
        
        matched_count = 0
        
        for yaml_file in pami_files:
            # Load the YAML file
            data = self.load_pami_yaml(str(yaml_file))
            
            if data is None:
                continue
            
            # Check if strategy_name matches
            file_strategy_name = data.get('strategy_name')
            pami_number = data.get('pami_number')
            sequence = data.get('sequence', [])
            
            if file_strategy_name != self.strategy_name:
                continue
            
            if pami_number is None:
                self.get_logger().warn(f"No pami_number found in {yaml_file.name}, skipping")
                continue
            
            matched_count += 1
            self.get_logger().info(f"\n{'='*60}")
            self.get_logger().info(f"✓ Matched: {yaml_file.name}")
            self.get_logger().info(f"  Strategy: {file_strategy_name}")
            self.get_logger().info(f"  PAMI Number: {pami_number}")
            self.get_logger().info(f"  Sequence Length: {len(sequence)} commands")
            
            # Create strategy topic name
            strategy_topic = f'/pami{pami_number}/strategy'
            start_topic = f'/pami{pami_number}/start'
            
            # Create strategy publisher if it doesn't exist
            if pami_number not in self.pami_publishers:
                self.pami_publishers[pami_number] = self.create_publisher(
                    Float32MultiArray,
                    strategy_topic,
                    10
                )
                self.get_logger().info(f"  Created publisher for topic: {strategy_topic}")
            
            # Create start publisher immediately (so topic exists from the beginning)
            if pami_number not in self.pami_start_publishers:
                self.pami_start_publishers[pami_number] = self.create_publisher(
                    Int32,
                    start_topic,
                    10
                )
                self.get_logger().info(f"  Created publisher for topic: {start_topic}")
            
            # Convert sequence to float array
            float_array = self.sequence_to_float_array(sequence)
            
            # Create and publish message
            msg = Float32MultiArray()
            msg.data = float_array
            
            # Publish the strategy
            self.pami_publishers[pami_number].publish(msg)
            
            self.get_logger().info(f"  Published {len(float_array)} floats to {strategy_topic}")
            self.get_logger().info(f"  ({len(sequence)} commands × 4 floats each)")
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"✓ Loaded and published {matched_count} matching PAMI strategies")
        self.get_logger().info(f"{'='*60}\n")
    
    def start_pami(self, number: int):
        """
        Send start signal to a specific PAMI robot
        
        Args:
            number: PAMI robot number (1, 2, 3, etc.)
        """
        topic_name = f'/pami{number}/start'
        
        # Create publisher if it doesn't exist
        if number not in self.pami_start_publishers:
            self.pami_start_publishers[number] = self.create_publisher(
                Int32,
                topic_name,
                10
            )
            self.get_logger().info(f"Created start publisher for topic: {topic_name}")
        
        # Create and publish message
        msg = Int32()
        msg.data = 1
        
        self.pami_start_publishers[number].publish(msg)
        self.get_logger().info(f"🚀 Sent start signal (1) to {topic_name}")
    
    def start_pamis(self):
        """
        Send start signal to all PAMI robots (PAMI 1 through 6)
        """
        self.get_logger().info("🚀 Starting all PAMIs...")
        
        for pami_number in range(1, 7):  # PAMIs 1 to 6
            self.start_pami(pami_number)
        
        self.get_logger().info("✅ Start signal sent to all 6 PAMIs")


def main(args=None):
    """
    Main function to run PAMI controller
    
    Usage:
        ros2 run euro pami_controller --ros-args -p strategy_name:=7etem -p pami_folder:=/path/to/pami_strategies
    """
    rclpy.init(args=args)
    
    # Create a temporary node to get parameters
    temp_node = rclpy.create_node('pami_controller_temp')
    
    # Declare parameters
    temp_node.declare_parameter('strategy_name', '')
    temp_node.declare_parameter('pami_folder', '')
    
    # Get parameter values
    strategy_name = temp_node.get_parameter('strategy_name').value
    pami_folder = temp_node.get_parameter('pami_folder').value
    
    # Destroy temp node
    temp_node.destroy_node()
    
    if not strategy_name:
        print("Error: strategy_name parameter is required!")
        print("Usage: ros2 run euro pami_controller --ros-args -p strategy_name:=7etem -p pami_folder:=/path/to/pami_strategies")
        rclpy.shutdown()
        return
    
    if not pami_folder:
        print("Error: pami_folder parameter is required!")
        print("Usage: ros2 run euro pami_controller --ros-args -p strategy_name:=7etem -p pami_folder:=/path/to/pami_strategies")
        rclpy.shutdown()
        return
    
    # Create PAMI controller
    pami_controller = PamiController(strategy_name, pami_folder)
    
    # Keep node alive for a bit to ensure messages are sent
    rclpy.spin_once(pami_controller, timeout_sec=1.0)
    
    # Cleanup
    pami_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
