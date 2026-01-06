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
from euro.strategy_executor import robot_status, stm32_task, front_task, back_task, pneumatic_task, IDLE, EMERGENCY_STOP

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
            'STM': deque(),
            'ESP_front': deque(),
            'ESP_back': deque(),
            'ESP_pneumatic': deque()
        }
        
        # Flag to pause task loading during pathfinding
        self.is_replanning = False
        
        # Global goal position storage
        self.global_pos_x = None
        self.global_pos_y = None
        self.global_pos_angle = None
        
        self.create_subscription(
            Vector3,
            '/robot_pose_raw',
            self.robot_pose_callback,
            10
        )
        self.pathfinder = PathfinderClient()
        self.goal = (0.2, 2.7, 1.57079632679)
        self.path_called = False
        self.get_logger().info("Subscribed to /robot_pose_raw")
        self.get_logger().info('Strategy Manager initialized')
    def robot_pose_callback(self, msg):
        self.latest_robot_pose = msg
        # self.get_logger().info(f"Robot pose updated: {msg}")
    def path(self, x_goal, y_goal, theta_goal):
        print("Path function called")
        """Create start and goal PoseStamped from current pose and goal"""
        # Start pose from latest robot pose
        if self.latest_robot_pose is None:
            self.get_logger().error("No robot pose received yet!")
            return
        
        # Set flag to prevent task loading during replanning
        self.is_replanning = True
        self.get_logger().info("🔄 Replanning path - pausing task execution...")
        
        # Clear the STM32 queue
        self.queues['STM'].clear()
        self.get_logger().info("🗑️  Cleared STM32 queue")
        
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
                self.queues['STM'].append({
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
            self.queues['STM'].append({
                'command': 'orientate',
                'params': {
                    'angle': theta_goal,
                    'is_radian': 0.0
                },
            })
        
        self.get_logger().info(f"✅ New path loaded with {len(self.queues['STM'])} commands")
        
        # Resume task execution
        self.is_replanning = False
        self.get_logger().info("▶️  Resuming task execution...")

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
            
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"QUEUE STATUS:")
        for device_name, queue in self.queues.items():
            self.get_logger().info(f"  {device_name}: {len(queue)} commands")
        self.get_logger().info(f"{'='*60}\n")
        
        return True
    
    def are_all_queues_empty(self) -> bool:
        """Check if all device queues are empty"""
        for queue in self.queues.values():
            if len(queue) > 0:
                return False
        return True
    
    def wait_for_all_queues_empty(self):
        """Wait until all device queues are empty (unless replanning)"""
        
        self.get_logger().info("⏳ Waiting for all commands to complete...")
        from euro.strategy_executor import IDLE
        while not self.are_all_queues_empty() or not IDLE:
            from euro.strategy_executor import IDLE
            # If replanning is happening, wait for it to finish
            if self.is_replanning:
                self.get_logger().info("   ⏸️  Paused - replanning in progress...")
                time.sleep(0.5)
                continue
            
            if not self.are_all_queues_empty():
                time.sleep(0.5)
            elif not IDLE:
                self.get_logger().info("   Waiting for robot to become idle...")
                self.get_logger().info(f"IDLE =  {IDLE} ")
                x=self.are_all_queues_empty()
                self.get_logger().info(f"queues empty: {x}")
                time.sleep(0.5)
        
        self.get_logger().info("✅ All commands completed!")
    
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
    import sys
    rclpy.init(args=args)
    
    # Import controllers
    from euro.stm32_controller import STM32Controller
    from euro.lidar import Lidar
    from euro.pathfinder_controller import PathfinderClient
    from euro.pneumatic_controller import PneumaticController
    from euro.front_systems_controller import FrontSystemsController
    from euro.back_systems_controller import BackSystemsController  
    # Initialize controllers
    stm32 = STM32Controller('rpi_stm32')
    lidar = Lidar()
    # Create strategy manager
    strategy = StrategyManager()
    front= FrontSystemsController()
    back= BackSystemsController()
    pneumatic= PneumaticController()
    # Create executor for ROS2 nodes
    executor = MultiThreadedExecutor()
    executor.add_node(stm32)
    executor.add_node(lidar)
    executor.add_node(strategy)
    executor.add_node(front)
    executor.add_node(back)
    executor.add_node(pneumatic)
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
    
    front_thread = threading.Thread(
        target=front_task,
        args=(strategy, back,pneumatic),  # TODO: Pass ESP32_1 controller when available
        daemon=True
    )
    front_thread.start()
    
    back_thread = threading.Thread(
        target=back_task,
        args=(strategy, back),  # TODO: Pass ESP32_2 controller when available
        daemon=True
    )
    back_thread.start()
    
    pneumatic_thread = threading.Thread(
        target=pneumatic_task,
        args=(strategy, pneumatic),  # TODO: Pass Pneumatic controller when available
        daemon=True
    )
    pneumatic_thread.start()
    
    strategy.get_logger().info("✓ All threads started\n")
    # Get tasks file path
    # Check if a YAML file path was provided as argument
    if len(sys.argv) > 1:
        tasks_file = sys.argv[1]
        strategy.get_logger().info(f"Using provided YAML file: {tasks_file}")
    else:
        # Default to tasks.yaml in package config directory
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
        
        # Execute tasks sequentially
        # Start a thread to execute tasks one by one
        def execute_tasks_sequentially():
            for task in strategy.tasks:
                # Extract and store global goal
                global_goal = task['global_goal']
                if global_goal:
                    strategy.global_pos_x = global_goal.get('x')
                    strategy.global_pos_y = global_goal.get('y')
                    
                    # Support both 'angle' and 'theta' keys
                    strategy.global_pos_angle = global_goal.get('theta') or global_goal.get('angle')
                    strategy.get_logger().info(
                        f"Global goal set: x={strategy.global_pos_x}, "
                        f"y={strategy.global_pos_y}, angle={strategy.global_pos_angle}°"
                    )
                else:
                    strategy.get_logger().warning(f"⚠️  No global goal found for task: {task['name']}")

                strategy.get_logger().info(f"\n{'='*60}")
                strategy.get_logger().info(f"▶️  STARTING TASK: {task['name']}")
                strategy.get_logger().info(f"{'='*60}\n")
                
                # Load task into queues
                strategy.load_task_to_queues(task['name'])
                
                # Wait for all commands to complete
                strategy.wait_for_all_queues_empty()
                
                strategy.get_logger().info(f"\n{'='*60}")
                strategy.get_logger().info(f"✅ COMPLETED TASK: {task['name']}")
                strategy.get_logger().info(f"{'='*60}\n")
            
            strategy.get_logger().info(f"\n{'='*80}")
            strategy.get_logger().info(f"🎉 ALL TASKS COMPLETED SUCCESSFULLY!")
            strategy.get_logger().info(f"{'='*80}\n")
        
        # Start task execution in a background thread
        task_executor_thread = threading.Thread(
            target=execute_tasks_sequentially,
            daemon=True
        )
        task_executor_thread.start()
        
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
