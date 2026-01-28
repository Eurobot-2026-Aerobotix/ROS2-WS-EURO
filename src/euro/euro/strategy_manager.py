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
from euro.strategy_executor import robot_status, stm32_task, front_task, back_task,rasp_pi_task, IDLE, EMERGENCY_STOP
import euro.strategy_executor as executor_module  # Import module to access SHUTDOWN_REQUESTED and other globals

# Global variable to store team color
color = None

class StrategyManager(Node):
    """
    Simple strategy manager that loads and displays tasks from YAML
    """
    
    def __init__(self, node_name='strategy_manager'):
        super().__init__(node_name)
        
        # Task storage - simple list
        self.tasks = []
        
        # Team color from YAML
        self.color = None
        
        # Strategy name from YAML
        self.strategy_name = None
        
        # Device queues
        self.queues = {
            'STM': deque(),
            'ESP_front': deque(),
            'ESP_back': deque(),
            'RASP_PI': deque()
        }
        
        # Flag to pause task loading during pathfinding
        self.is_replanning = False
        # Global goal position storage
        self.global_pos_x = None
        self.global_pos_y = None
        self.global_pos_angle = None
        self.latest_robot_pose = None
        self.create_subscription(
            Vector3,
            '/robot_pose_raw',
            self.robot_pose_callback,
            10
        )
        self.pathfinder = PathfinderClient()
        self.goal = (0.2, 2.7, 1.57079632679)
        self.path_called = False
    def robot_pose_callback(self, msg):
        self.latest_robot_pose = msg
    def path(self, x_goal, y_goal, theta_goal):
        print("Path function called")
        """Create start and goal PoseStamped from current pose and goal"""
        # Start pose from latest robot pose
        if self.latest_robot_pose is None:
            self.get_logger().error("No robot pose received yet!")
            return
        
        # Set flag to prevent task loading during replanning
        self.is_replanning = True
        
        # Clear the STM32 queue
        self.queues['STM'].clear()
        
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
            values = cmd.replace('go_to(', '').replace(')', '').split(',')
            x = float(values[0].strip())
            y = float(values[1].strip())
            speed = int(values[2].strip())
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
        
        # Resume task execution
        self.is_replanning = False

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
        
        # Iterate through sequence and add to queues
        for idx, cmd in enumerate(task['sequence']):
            device = cmd.get('device')
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            self.get_logger().info(f"⚙️  [{idx+1}/{len(task['sequence'])}] {device} -> {command}")
            executor_module.SYSTEMS_IDLE_COUNT=0
            # Check if next command is pick_up for ESP_front or ESP_back
            if idx + 1 < len(task['sequence']):
                next_cmd = task['sequence'][idx + 1]
                next_device = next_cmd.get('device')
                next_command = next_cmd.get('command')
                
                if next_command in ['pick_up', 'put_down'] and next_device in ['ESP_front', 'ESP_back']:
                    prepare_cmd = {
                        'device': next_device,
                        'command': 'prepare',
                        'params': {'action': next_command},
                        'wait': False
                    }
                    self.queues[next_device].append(prepare_cmd)
                    self.get_logger().info(f"      🔧 Adding prepare command for {next_device} with action: {next_command}")

            # Add command to device queue
            if device in self.queues:
                if(device == "ESP_front" ) :
                    executor_module.ESP_FRONT_IDLE=False
                if(device == "ESP_back" ) :
                    executor_module.ESP_BACK_IDLE=False
                if(device == "STM"):
                    executor_module.IDLE_FOR_SYSTEMS=False
                    
                if wait :
                    self.get_logger().info(f"      ⏳ Waiting for dependencies...")
                    if(device == "STM"):
                        while not executor_module.ESP_FRONT_IDLE or not executor_module.ESP_BACK_IDLE:
                            time.sleep(0.1)
                    if(device == "ESP_front" or device == "ESP_back"):
                        executor_module.SYSTEMS_IDLE_COUNT=0
                        n=len((self.queues["STM"]))
                        if executor_module.IDLE_FOR_SYSTEMS and not executor_module.IDLE:
                            executor_module.SYSTEMS_IDLE_COUNT=1
                        while executor_module.SYSTEMS_IDLE_COUNT !=n :
                            self.get_logger().info(f"idle count : {executor_module.SYSTEMS_IDLE_COUNT} n: {n}")
                            time.sleep(0.1)
                        self.get_logger().info(f"idle count : {executor_module.SYSTEMS_IDLE_COUNT} n: {n}")
                if device =="STM" and command =="set_position":
                    self.queues["RASP_PI"].append(cmd)
                else:
                    if device =="RASP_PI" and command=="Wait for manual start":
                        executor_module.WAITING_TIRETTE=True
                    self.queues[device].append(cmd)
                
            else:
                self.get_logger().warn(f"      ⚠ Unknown device: {device}")
        
        return True
    
    def are_all_queues_empty(self) -> bool:
        """Check if all device queues are empty"""
        for queue in self.queues.values():
            if len(queue) > 0:
                return False
        return True
    
    def wait_for_all_queues_empty(self):
        """Wait until all device queues are empty (unless replanning)"""
        
        while (not (self.are_all_queues_empty() or(  executor_module.ESP_FRONT_IDLE==True and executor_module.ESP_BACK_IDLE==True ))) or (not executor_module.IDLE) or executor_module.WAITING_TIRETTE or len(self.queues["STM"])!=0 or len(self.queues["RASP_PI"])!=0  :
            # Check if shutdown is requested
            if executor_module.SHUTDOWN_REQUESTED:
                self.get_logger().info("Shutdown requested, stopping queue wait")
                return
            
            # If replanning is happening, wait for it to finish
            if self.is_replanning:
                time.sleep(0.5)
                continue
            if not self.are_all_queues_empty():
                time.sleep(0.1)
            elif not executor_module.IDLE:
                time.sleep(0.1)
    
    def display_queue_contents(self):
        """Display the full contents of all queues with parameters"""
        pass

def reset_systems(pneumatic_controller,front_controller,back_controller,stm32_controller=None,strategy=None,immediate=False,xd=True):
    """Reset all systems - can queue or execute immediately"""
    if strategy and not immediate:
        # Add RESET command to RASP_PI queue for background execution
        strategy.queues['RASP_PI'].append({
            'command': 'RESET',
            'params': {},
            'wait': False
        })
    else:
        # Execute reset commands immediately (for shutdown or when strategy not available)
        if stm32_controller:
            if xd:
                stm32_controller.stop(timeout=2.0)
                time.sleep(0.5)
            #stm32_controller.reset(timeout=2.0)
        
        # Reset pneumatic and ESP controllers
        pneumatic_controller.front(16)
        pneumatic_controller.back(16)
        front_controller.kabech(0)
        back_controller.kabech(0)
        front_controller.dawer(0)
        back_controller.dawer(0)
        front_controller.n20(0)
        back_controller.n20(0)
        # Allow time for messages to be sent
        time.sleep(0.2)
    
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
    from euro.rpi_controller import RPiController
    # Initialize controllers
    
    lidar = Lidar()
    # Create strategy manager
    strategy = StrategyManager()
   
    front= FrontSystemsController()
    back= BackSystemsController()
    pneumatic= PneumaticController()
    rpi= RPiController()
    # Create executor for ROS2 nodes
    executor = MultiThreadedExecutor()
    executor.add_node(lidar)
    executor.add_node(strategy)
    executor.add_node(front)
    executor.add_node(back)
    executor.add_node(pneumatic)
    executor.add_node(rpi)
    rpi.reset_stm32()
    time.sleep(1)
    stm32 = STM32Controller('rpi_stm32')
    executor.add_node(stm32)
    strategy.get_logger().info("Waiting for STM32 service to become available...")
    if stm32.navi_client.wait_for_service(timeout_sec=20.0):
        strategy.get_logger().info("✅ STM32 service is ready!")
    else:
        strategy.get_logger().error("❌ STM32 service not available after 20 seconds - robot may not respond to commands")
     # Start background threads for each device
    strategy.get_logger().info("Starting background threads...")
    time.sleep(1)
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
        args=(strategy, front,pneumatic),  
        daemon=True
    )
    front_thread.start()
    
    back_thread = threading.Thread(
        target=back_task,
        args=(strategy, back,pneumatic), 
        daemon=True
    )
    back_thread.start()
    
    
    rasp_pi_thread = threading.Thread(
        target=rasp_pi_task,
        args=(strategy, rpi, stm32, front, back, pneumatic),
        daemon=True
    )
    rasp_pi_thread.start()
    
    # Get tasks file path
    # Check if a YAML file path was provided as argument
    if len(sys.argv) > 1:
        tasks_file = sys.argv[1]
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
        strategy.tasks, strategy.color, strategy.strategy_name = load_tasks_from_yaml(tasks_file)
        
        # Load and publish PAMI strategies
        pami_folder = os.path.join(os.path.dirname(tasks_file), '..', 'pami_strategies')
        pami_folder = os.path.abspath(pami_folder)
        
        if os.path.exists(pami_folder):
            from euro.pami_controller import PamiController
            pami_ctrl = PamiController(strategy.strategy_name, pami_folder, 'pami_controller_internal')
        
        reset_systems(pneumatic,front,back,stm32,strategy,True,False)
        time.sleep(2)
        # Execute tasks sequentially
        # Start a thread to execute tasks one by one
        def execute_tasks_sequentially():
            for task in strategy.tasks:
                # Check if shutdown was requested
                if executor_module.SHUTDOWN_REQUESTED:
                    strategy.get_logger().info("Shutdown requested, stopping task execution")
                    return
                
                # Extract and store global goal
                global_goal = task['global_goal']
                if global_goal:
                    strategy.global_pos_x = global_goal.get('x')
                    strategy.global_pos_y = global_goal.get('y')
                    
                    # Support both 'angle' and 'theta' keys
                    strategy.global_pos_angle = global_goal.get('theta') or global_goal.get('angle')
                else:
                    strategy.get_logger().warning(f"⚠️  No global goal found for task: {task['name']}")
                
                # Load task into queues
                strategy.load_task_to_queues(task['name'])
                
                # Wait for all commands to complete
                strategy.wait_for_all_queues_empty()
        
        # Start task execution in a background thread
        task_executor_thread = threading.Thread(
            target=execute_tasks_sequentially,
            daemon=True
        )
        task_executor_thread.start()
        
    except Exception as e:
        strategy.get_logger().error(f"Failed: {str(e)}")
        return
    
   
    
    # Keep node alive with 'q' key shutdown
    import sys
    import select
    import termios
    import tty
    
    # Check if stdin is a TTY before configuring terminal settings
    is_tty = sys.stdin.isatty()
    old_settings = None
    
    if is_tty:
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        # Set terminal to raw mode to read single characters
        tty.setcbreak(sys.stdin.fileno())
    
    try:
        shutdown_requested = False
        while rclpy.ok() and not shutdown_requested:
            # Spin once to process callbacks
            executor.spin_once(timeout_sec=0.1)
            # Check if 'q' was pressed
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'q':
                    shutdown_requested = True
                    
                    # Stop STM32 and reset all systems immediately (not queued)
                    try:
                        strategy.get_logger().info("Stopping and resetting all systems...")
                        reset_systems(pneumatic,front,back,stm32,strategy,immediate=True)
                        time.sleep(0.5)
                    except Exception as e:
                        strategy.get_logger().error(f"Failed to stop/reset systems: {e}")
                    
                
                    
                    break
    
    except KeyboardInterrupt:
        pass
    finally:
        # Signal threads to stop first
        strategy.get_logger().info("Signaling threads to stop...")
        executor_module.SHUTDOWN_REQUESTED = True
        
        # Give threads time to react to shutdown flag
        time.sleep(0.3)
        
        # Wait for threads to finish (with longer timeout)
        threads = [
            ('Status Monitor', status_thread),
            ('STM32 Task', stm32_thread),
            ('Front Task', front_thread),
            ('Back Task', back_thread),
            ('RaspPi Task', rasp_pi_thread),
            ('Task Executor', task_executor_thread)
        ]
        
        for name, thread in threads:
            if thread.is_alive():
                strategy.get_logger().info(f"Waiting for {name} thread to stop...")
                thread.join(timeout=5.0)
                if thread.is_alive():
                    strategy.get_logger().warning(f"Thread {name} did not stop in time (still blocked)")
                else:
                    strategy.get_logger().info(f"✓ {name} thread stopped")
        
        # Restore terminal settings if they were changed
        if old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except:
                pass  # Terminal may already be closed
        
        # Cleanup nodes
        try:
            # Clean up GPIO resources after threads are stopped
            rpi.cleanup()
            
            # Destroy all nodes
            strategy.destroy_node()
            stm32.destroy_node()
            lidar.destroy_node()
            front.destroy_node()
            back.destroy_node()
            pneumatic.destroy_node()
            rpi.destroy_node()
            executor.shutdown()
        except Exception as e:
            strategy.get_logger().warning(f"Cleanup warning: {e}")
        
        # Only shutdown if context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()