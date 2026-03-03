#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from std_srvs.srv import Trigger
import yaml
import os
import time
import threading
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
        
        # Dictionary to store service clients for status checking
        self.pami_status_clients = {}
        
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
    
    def sequence_to_float_array(self, sequence: List[List], pami_number: int = None) -> List[float]:
        """
        Convert sequence list to flat float array with padding
        For pami 1-6: Each command is padded to exactly 4 floats
        For pami 7: Sends floats as-is without padding or trimming
        
        Args:
            sequence: List of lists containing command data
            pami_number: PAMI robot number (used to determine encoding logic)
            
        Returns:
            Flattened list of floats
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
            
            # For pami 7: send floats as-is without padding/trimming
            flat_array.extend(cmd_floats)        
        return flat_array
    
    def check_pami_status(self, pami_number: int) -> bool:
        """
        Check the status of a PAMI by calling its /pamiX/status service
        
        Args:
            pami_number: PAMI robot number
            
        Returns:
            True if service returns success=True, False otherwise
        """
        service_name = f'/pami{pami_number}/status'
        
        # Create service client if it doesn't exist
        if pami_number not in self.pami_status_clients:
            self.pami_status_clients[pami_number] = self.create_client(Trigger, service_name)
        
        client = self.pami_status_clients[pami_number]
        
        # Wait for service to be available (with timeout)
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Service {service_name} not available")
            return False
        
        # Call the service
        request = Trigger.Request()
        future = client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            try:
                response = future.result()
                return response.success
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return False
        else:
            self.get_logger().warn(f"Service call to {service_name} timed out")
            return False
    
    def publish_strategy_with_retry(self, pami_number: int, msg: Float32MultiArray, max_retries: int = 10):
        """
        Publish strategy and retry until status returns True
        
        Args:
            pami_number: PAMI robot number
            msg: Strategy message to publish
            max_retries: Maximum number of retry attempts
        """
        strategy_topic = f'/pami{pami_number}/strategy'
        
        for attempt in range(1, max_retries + 1):
            # Publish the strategy
            self.pami_publishers[pami_number].publish(msg)
            self.get_logger().info(f"  📤 Attempt {attempt}: Published strategy to {strategy_topic}")
            
            # Wait a bit for the message to be processed
            time.sleep(0.2)
            
            # Check status
            status = self.check_pami_status(pami_number)
            
            if status:
                self.get_logger().info(f"  ✅ PAMI {pami_number} confirmed strategy received (status=True)")
                return True
            else:
                self.get_logger().warn(f"  ⚠️ PAMI {pami_number} status returned False, retrying...")
                time.sleep(0.3)
        
        self.get_logger().error(f"  ❌ Failed to confirm strategy for PAMI {pami_number} after {max_retries} attempts")
        return False
    
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
            float_array = self.sequence_to_float_array(sequence, pami_number)
            
            # Create and publish message
            msg = Float32MultiArray()
            msg.data = float_array
            self.get_logger().info(f"  Strategy: {len(float_array)} floats ({len(sequence)} commands, raw encoding)")  
            # Publish the strategy with retry logic
            self.publish_strategy_with_retry(pami_number, msg)
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"✓ Loaded and published {matched_count} matching PAMI strategies")
        self.get_logger().info(f"{'='*60}\n")
    
    def start_pami(self, number: int, max_retries: int = 3):
        """
        Send start signal to a specific PAMI robot and retry until status returns False
        
        Args:
            number: PAMI robot number (1, 2, 3, etc.)
            max_retries: Maximum number of retry attempts
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
        
        for attempt in range(1, max_retries + 1):
            # Publish start signal
            self.pami_start_publishers[number].publish(msg)
            self.get_logger().info(f"🚀 Attempt {attempt}: Sent start signal (1) to {topic_name}")
            
            # Wait a bit for the message to be processed
            time.sleep(0.2)
            
            # Check status (should be False for start to be successful)
            status = self.check_pami_status(number)
            
            if not status:  # Status should be False
                self.get_logger().info(f"✅ PAMI {number} confirmed start (status=False)")
                return True
            else:
                self.get_logger().warn(f"⚠️ PAMI {number} status returned True, retrying start...")
                time.sleep(0.3)
        
        self.get_logger().error(f"❌ Failed to confirm start for PAMI {number} after {max_retries} attempts")
        return False
    
    def start_pamis(self):
        """
        Send start signal to all PAMI robots (PAMI 1 through 6) in parallel threads
        """
        self.get_logger().info("🚀 Starting all PAMIs in parallel...")
        
        threads = []
        for pami_number in range(1, 8):  # PAMIs 1 to 6
            thread = threading.Thread(
                target=self.start_pami,
                args=(pami_number,),
                daemon=True,
                name=f"PAMI_{pami_number}_start"
            )
            thread.start()
            threads.append(thread)
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join(timeout=5.0)  # 5 second timeout per PAMI
        
        self.get_logger().info("✅ All PAMI start threads completed")


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
