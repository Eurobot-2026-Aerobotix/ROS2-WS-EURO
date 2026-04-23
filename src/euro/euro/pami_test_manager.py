#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import argparse
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
        self.strategy_color = None
        self.selected_pamis = []
        self.target_order = 1
        
        # PAMI controller instance
        self.pami_controller = None
        
        self.get_logger().info('PAMI Test Manager initialized')

    def _parse_pami_numbers(self, pami_numbers_input):
        """
        Parse PAMI numbers from a comma-separated string or list into sorted unique [1..7].
        """
        if pami_numbers_input is None:
            return []

        if isinstance(pami_numbers_input, str):
            raw_values = [v.strip() for v in pami_numbers_input.split(',') if v.strip()]
        else:
            raw_values = list(pami_numbers_input)

        parsed = set()
        invalid = []
        for value in raw_values:
            try:
                number = int(value)
                if 1 <= number <= 7:
                    parsed.add(number)
                else:
                    invalid.append(str(value))
            except (TypeError, ValueError):
                invalid.append(str(value))

        if invalid:
            self.get_logger().warn(f"Ignoring invalid PAMI numbers: {', '.join(invalid)} (valid range is 1..7)")

        return sorted(parsed)

    def _resolve_pami_folder(self, strategy_input: str):
        """
        Resolve pami_strategies folder from strategy file path or known dashboard locations.
        """
        if strategy_input and os.path.exists(strategy_input):
            candidate = os.path.abspath(os.path.join(os.path.dirname(strategy_input), '..', 'pami_strategies'))
            if os.path.exists(candidate):
                return candidate

        candidates = [
            os.path.abspath(os.path.join(os.getcwd(), 'pami_strategies')),
            os.path.abspath(os.path.join(os.path.expanduser('~'), 'EUROBOT', 'Eurobot_Dashboard', 'pami_strategies')),
        ]

        for folder in candidates:
            if os.path.exists(folder):
                return folder

        return None

    def _infer_color_from_name(self, name: str):
        low = str(name or '').lower()
        if 'yellow' in low or 'jaune' in low or 'asfer' in low:
            return 'yellow'
        if 'blue' in low or 'azreg' in low:
            return 'blue'
        return None
    
    def load_strategy_info(self, strategy_input: str):
        """
        Load strategy information from YAML file or directly from strategy name.
        
        Args:
            strategy_input: Path to strategy YAML file or strategy name
        """
        if not strategy_input:
            self.get_logger().error("Empty strategy input")
            return False

        # If a direct strategy name is provided, use it as-is.
        if os.path.sep not in strategy_input and not strategy_input.endswith('.yaml'):
            inferred_name = strategy_input.strip()
            if not inferred_name:
                self.get_logger().error("Could not infer strategy name from input")
                return False

            self.strategy_name = inferred_name
            self.strategy_color = self._infer_color_from_name(inferred_name)
            self.get_logger().info(
                f"Using provided strategy_name='{self.strategy_name}', color='{self.strategy_color}'"
            )
            return True

        if not os.path.exists(strategy_input):
            # New PAMI-only flow: allow passing a non-existing strategies/<name>.yaml path.
            # We infer the strategy name from filename and continue.
            inferred_name = os.path.splitext(os.path.basename(strategy_input))[0]
            if not inferred_name:
                self.get_logger().error(f"Strategy file not found and could not infer strategy name: {strategy_input}")
                return False

            self.strategy_name = inferred_name
            self.strategy_color = self._infer_color_from_name(inferred_name)
            self.get_logger().warning(
                f"Strategy file not found: {strategy_input}. "
                f"Using inferred strategy_name='{self.strategy_name}', color='{self.strategy_color}'"
            )
            return True
        
        try:
            with open(strategy_input, 'r') as file:
                data = yaml.safe_load(file)
                
                # Get strategy name (try both 'name' and 'strategy_name' fields)
                self.strategy_name = data.get('name') or data.get('strategy_name')
                self.strategy_color = data.get('color')
                if not self.strategy_color:
                    self.strategy_color = self._infer_color_from_name(self.strategy_name)
                
                if not self.strategy_name:
                    self.get_logger().error("No 'name' or 'strategy_name' found in YAML file")
                    return False
                
                self.get_logger().info(f"✓ Loaded strategy: {self.strategy_name}")
                return True
                
        except Exception as e:
            self.get_logger().error(f"Error loading strategy file: {e}")
            return False
    
    def load_pami_strategies(self, strategy_input: str):
        """
        Load PAMI strategies.
        
        Args:
            strategy_input: Path to strategy YAML file or strategy name
        """
        pami_folder = self._resolve_pami_folder(strategy_input)
        
        if not pami_folder or not os.path.exists(pami_folder):
            self.get_logger().warn(f"PAMI strategies folder not found: {pami_folder}")
            return False
        
        self.get_logger().info(f"Loading PAMI strategies from: {pami_folder}")
        
        # Create PAMI controller
        self.pami_controller = PamiController(
            self.strategy_name, 
            pami_folder, 
            'pami_controller_test',
            team_color=self.strategy_color,
            target_order=self.target_order,
            selected_pamis=self.selected_pamis,
            strict_strategy_match=True,
        )
        
        return True
    
    def start_all_pamis(self):
        """
        Send start signal to all PAMI robots
        """
        if self.pami_controller:
            if self.selected_pamis:
                self.get_logger().info(f"🚀 Starting selected PAMIs: {self.selected_pamis}")
            else:
                self.get_logger().info("🚀 Starting all PAMIs...")
            self.pami_controller.start_pamis(self.selected_pamis)
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
        ros2 run euro pami_test_manager <strategy_name_or_file_path> [pami_numbers]
    
    Example:
        ros2 run euro pami_test_manager /path/to/strategies/7etem.yaml
        ros2 run euro pami_test_manager pami_complete_yellow 2,3,5
    """
    rclpy.init(args=args)
    
    # Create PAMI test manager
    manager = PamiTestManager()
    
    # Create executor for ROS2 node
    executor = MultiThreadedExecutor()
    executor.add_node(manager)
    
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('strategy_input', nargs='?')
    parser.add_argument('pami_numbers', nargs='?', default='')
    parser.add_argument('target_order', nargs='?', default='1')

    cli_args, _ = parser.parse_known_args(sys.argv[1:])

    # Get strategy input from command line arguments
    strategy_input = cli_args.strategy_input
    if strategy_input:
        strategy_input = strategy_input.strip()

    if not strategy_input:
        manager.get_logger().error("No strategy input provided!")
        manager.get_logger().info("Usage: ros2 run euro pami_test_manager <strategy_name_or_file_path> [pami_numbers] [target_order]")
        manager.get_logger().info("Example: ros2 run euro pami_test_manager pami_complete_yellow 2,3,5 2")
        rclpy.shutdown()
        return

    manager.selected_pamis = manager._parse_pami_numbers(cli_args.pami_numbers)
    try:
        manager.target_order = int(cli_args.target_order)
    except Exception:
        manager.target_order = 1
    
    # Load strategy information
    if not manager.load_strategy_info(strategy_input):
        manager.get_logger().error("Failed to load strategy information")
        rclpy.shutdown()
        return
    
    # Load PAMI strategies
    if not manager.load_pami_strategies(strategy_input):
        manager.get_logger().error("Failed to load PAMI strategies")
        rclpy.shutdown()
        return
    
    manager.get_logger().info("\n" + "="*60)
    manager.get_logger().info("PAMI Test Manager Ready!")
    manager.get_logger().info(f"Strategy: {manager.strategy_name}")
    manager.get_logger().info(f"Target order: {manager.target_order}")
    manager.get_logger().info(f"PAMIs: {manager.selected_pamis if manager.selected_pamis else 'all (1..7)'}")
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
