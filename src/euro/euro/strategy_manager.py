#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import os
import time
import threading
from typing import List, Dict, Any
from collections import deque
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion, Vector3, Point
from aruco_mapper_msgs.msg import ArucoMapperState
from euro.yaml_loader import load_tasks_from_yaml
import math
from euro.pathfinder_controller import PathfinderClient
from euro.strategy_executor import robot_status, lidar_status, stm32_task, front_task, back_task,rasp_pi_task, path_planning_task, IDLE, EMERGENCY_STOP
import euro.strategy_executor as executor_module  # Import module to access SHUTDOWN_REQUESTED and other globals

# Global variable to store team color
color = None
n=0
found_goal_batch=False
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
        self.latest_robot_pose_beacon = None
        self.latest_robot_pose_beacon_count = 0
        self.end_game=False
        # Track last pick_up/put_down command and device from task
        self.last_task_command = None
        self.last_task_device = None
        # Storage for prepared path commands
        self.prepared_commands = []
        self.prepared_goal_angle = None
        # Create Round timer instance
        from euro.strategy_executor import Round
        self.round = Round()
        
        self.create_subscription(
            Vector3,
            '/robot_pose_raw',
            self.robot_pose_callback,
            10
        )
        self.create_subscription(
            ArucoMapperState,
            '/aruco_mapper/state_robots',
            self.aruco_mapper_callback,
            10
        )
        self.pathfinder = PathfinderClient()
        self.path_called = False
    def robot_pose_callback(self, msg):
        self.latest_robot_pose = msg
    
    def aruco_mapper_callback(self, msg):
        """Callback for aruco mapper state - store entire message"""
        self.latest_robot_pose_beacon = msg
        #self.get_logger().info(f"aruco mapper callback")
        self.latest_robot_pose_beacon_count += 1
    def path(self, end_game=False, prepare=False):
        """Plan path from current position to stored goal.
        
        Args:
            end_game: Whether this is for end game return home
            prepare: If True, generate and store path without loading to queue.
                    If False, load previously prepared path to queue without generating new path.
        
        Returns:
            bool: Success status
        """
        global n 
        global found_goal_batch
        
        if prepare:
            # PREPARE MODE: Generate path and store it, don't load to queue yet
            self.get_logger().info(f"🗺️  Preparing path (searching only, not loading to queue)")
            
            # Use pathfinder's path method (it uses stored goal and latest robot pose)
            stm32_commands, success = self.pathfinder.path()
            
            if success:
                # Store commands and goal angle for later execution
                self.prepared_commands = stm32_commands
                self.prepared_goal_angle = self.pathfinder.goal_angle
                self.get_logger().info(f"✅ Path prepared! {len(stm32_commands)} waypoints found (not loaded to queue yet)")
            else:
                self.prepared_commands = []
                self.prepared_goal_angle = None
                self.get_logger().warning("⚠️ Path preparation failed")
            
            return success
        
        else:
            # EXECUTE MODE: Load prepared path to queue (or generate if none prepared)
            self.get_logger().info(f"🚀 Loading path to queue for execution")
            
            # Set flag to prevent task loading during replanning
            self.is_replanning = True
            
            
            # Check if we have a prepared path
            if self.prepared_commands:
                self.get_logger().info(f"✓ Using prepared path with {len(self.prepared_commands)} waypoints")
                stm32_commands = self.prepared_commands
                goal_angle = self.prepared_goal_angle
                success = True
                # Clear prepared commands after use
                self.prepared_commands = []
                self.prepared_goal_angle = None
            else:
                # No prepared path - generate one now (fallback)
                self.get_logger().warning("⚠️ No prepared path available - generating now as fallback")
                stm32_commands, success = self.pathfinder.path()
                goal_angle = self.pathfinder.goal_angle if success else None
            
            # Load commands to queue
            if success:
                # Clear the STM32 queue
                self.queues['STM'].clear()
                executor_module.SYSTEMS_IDLE_COUNT=0
                # Process commands and add to queue
                last_cmd = None
                for cmd in stm32_commands:
                    values = cmd.replace('go_to(', '').replace(')', '').split(',')
                    x = float(values[0].strip())
                    y = float(values[1].strip())
                    speed = int(values[2].strip())
                    if cmd != last_cmd:
                        if self.last_task_device=="ESP_back":
                            self.get_logger().info(f"go_to_backwards: x={x}, y={y}")
                            self.queues['STM'].append({
                                'command': 'go_to_backwards',
                                'params': [x, y, 1100.0, {'from_pathfinding': True}],  # Mark as from pathfinding
                                'wait': False
                            })
                        else:
                            self.get_logger().info(f"go_to: x={x}, y={y}")
                            self.queues['STM'].append({
                                'command': 'go_to',
                                'params': [x, y, 1100.0, {'from_pathfinding': True}],  # Mark as from pathfinding
                                'wait': False
                            })

                    last_cmd = cmd
                    
                if success and goal_angle is not None and not end_game:
                    self.queues['STM'].append({
                        'command': 'orientate',
                        'params': [goal_angle, 0.0],  # List format: [angle, is_radian]
                    })
                    if self.last_task_command=='pick_up' and found_goal_batch:
                        if self.last_task_device == 'ESP_front' :
                            self.queues['STM'].append({
                                'command': 'go_to',
                                'params': [self.global_pos_x, self.global_pos_y, 1100.0, 1.0, {'from_pathfinding': True}],  # Mark as from pathfinding
                                'wait': False
                            })
                            self.get_logger().info(f"Added go_to command with global position: x={self.global_pos_x}, y={self.global_pos_y} (last command after pathfinding )")
                        elif self.last_task_device == 'ESP_back' :
                            self.queues['STM'].append({
                                'command': 'go_to_backwards',
                                'params': [self.global_pos_x, self.global_pos_y, 1100.0, 1.0, {'from_pathfinding': True}],  # Mark as from pathfinding
                                'wait': False
                            })
                            
                n=len((self.queues["STM"]))
                executor_module.SYSTEMS_IDLE_COUNT=0
                self.get_logger().info(f"✅ Path loaded to queue! {len(self.queues['STM'])} commands ready for execution")
                # Resume task execution
                executor_module.replanning = False
                self.is_replanning = False
                
            return success
    def handle_global_goal(self, x, y, angle):
        global found_goal_batch
        pos = Point()
        pos.x = float(x)
        pos.y = float(y)
        pos.z = 0.0
        known_id = ''
        self.get_logger().info("requesting from pathfinder for checking if there is a batch or no(global pose ) ")
        color_pattern, success, direction, batch_center, batch_known_id = self.pathfinder.get_batch_color(known_id, pos)
        if success and self.last_task_command =='pick_up' :
            found_goal_batch=True
            if (direction == "horizontal" and y < batch_center.y) :
                global_x=x
                global_y=y-0.1
                global_angle=angle
            elif(direction == "horizontal" and y > batch_center.y) :
                global_x=x
                global_y=y+0.1
                global_angle=angle
            elif(direction == "vertical" and x < batch_center.x) :
                global_x=x-0.1
                global_y=y
                global_angle=angle
            elif(direction == "vertical" and x > batch_center.x) :
                global_x=x+0.1
                global_y=y
                global_angle=angle
            else:
                # Default case: use original coordinates
                global_x=x
                global_y=y
                global_angle=angle
                self.get_logger().warn(f"Edge case: position equals batch center or unexpected direction '{direction}'")
            self.pathfinder.set_goal(float(global_x), float(global_y), float(global_angle))
        else:
            found_goal_batch=False
            self.get_logger().info(f"pathfinder goal set to :x = {x}, y = {y}  angle=  {angle}without pickuo ")
            self.pathfinder.set_goal(float(x), float(y), float(angle))
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
        global n
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
            if not self.end_game:
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
                    if(device == "STM" and command !="set_position"):
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
                                self.get_logger().debug(f"Waiting for STM commands to be sent... STM queue size: {n}, SYSTEMS_IDLE_COUNT: {executor_module.SYSTEMS_IDLE_COUNT}")
                                time.sleep(0.1)
                    if device =="STM" and command =="set_position":
                        self.queues["RASP_PI"].append(cmd)
                    else:
                        if device =="RASP_PI" and command=="Wait for manual start":
                            executor_module.WAITING_TIRETTE=True
                        if not self.end_game:
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
        
        while (not (self.are_all_queues_empty() or(  executor_module.ESP_FRONT_IDLE==True and executor_module.ESP_BACK_IDLE==True ))) or (not executor_module.IDLE) or executor_module.WAITING_TIRETTE or len(self.queues["STM"])!=0 or len(self.queues["RASP_PI"])!=0 or executor_module.replanning or self.end_game==True:
            # Check if shutdown is requested
            self.get_logger().debug(f"Waiting for queues to empty... STM queue size: {len(self.queues['STM'])}, RASP_PI queue size: {len(self.queues['RASP_PI'])}, ESP_FRONT_IDLE: {executor_module.ESP_FRONT_IDLE}, ESP_BACK_IDLE: {executor_module.ESP_BACK_IDLE}, IDLE: {executor_module.IDLE}, WAITING_TIRETTE: {executor_module.WAITING_TIRETTE}, replanning: {executor_module.replanning}")
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
    executor.add_node(strategy.pathfinder)
    
    # Reset STM32 and wait for reconnection
    strategy.get_logger().info("Resetting STM32...")
    rpi.reset_stm32()
    
    # Wait longer for STM32 to boot and connect to micro-ROS agent
    strategy.get_logger().info("Waiting for STM32 to boot and connect to micro-ROS agent...")
    
    stm32 = STM32Controller('rpi_stm32')
    executor.add_node(stm32)
    
    # Wait for STM32 service to become available
    strategy.get_logger().info("Waiting for STM32 service to become available...")
    if stm32.navi_client.wait_for_service(timeout_sec=20.0):
        strategy.get_logger().info("✅ STM32 service is ready!")
    else:
        strategy.get_logger().error("❌ STM32 service not available after 20 seconds - robot may not respond to commands")
    
     # Start background threads for each device
    strategy.get_logger().info("Starting background threads...")
    
    # Reset ArUco stability in background thread (after executor starts spinning)
    def reset_aruco_stability():
        time.sleep(0.5)  # Wait for executor to start
        strategy.pathfinder.reset_stability()
        strategy.pathfinder.clear_map()
    
    reset_thread = threading.Thread(
        target=reset_aruco_stability,
        daemon=True
    )
    reset_thread.start()
    
    status_thread = threading.Thread(
        target=robot_status,
        args=(strategy, stm32, lidar),
        daemon=True
    )
    status_thread.start()
    
    lidar_thread = threading.Thread(
        target=lidar_status,
        args=(strategy, stm32, lidar, strategy.round),
        daemon=True
    )
    lidar_thread.start()
    
    path_planning_thread = threading.Thread(
        target=path_planning_task,
        args=(strategy, lidar, strategy.round, pneumatic, front, back),
        daemon=True
    )
    path_planning_thread.start()
    
    stm32_thread = threading.Thread(
        target=stm32_task,
        args=(strategy, stm32,lidar),
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
    
    # Get tasks file path and load PAMI controller before starting rasp_pi_thread
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
            #pami_ctrl = PamiController(strategy.strategy_name, pami_folder, 'pami_controller_internal')
            pami_ctrl = None
        else:
            pami_ctrl = None
        
        # Start rasp_pi_thread with pami_ctrl
        rasp_pi_thread = threading.Thread(
            target=rasp_pi_task,
            args=(strategy, strategy.round, rpi, stm32, front, back, pneumatic, pami_ctrl),
            daemon=True
        )
        rasp_pi_thread.start()
        #pami_ctrl.load_and_publish_pami_strategies()
        
        # Spin executor briefly to initialize publishers before sending reset
        for _ in range(5):
            executor.spin_once(timeout_sec=0.01)
        
        reset_systems(pneumatic,front,back,None,strategy,True,False)
        time.sleep(2)
        strategy.pathfinder.update_map()
        # Execute tasks sequentially
        # Start a thread to execute tasks one by one
        def execute_tasks_sequentially():
            for task in strategy.tasks:
                # Check if shutdown was requested
                if executor_module.SHUTDOWN_REQUESTED:
                    strategy.get_logger().info("Shutdown requested, stopping task execution")
                    return
                
                # Check last command in task sequence
                if task.get('sequence') and len(task['sequence']) > 0:
                    last_cmd = task['sequence'][-1]
                    last_command = last_cmd.get('command')
                    last_device = last_cmd.get('device')
                    
                    if last_command in ['pick_up', 'put_down'] and last_device in ['ESP_front', 'ESP_back']:
                        strategy.get_logger().info(f"📋 Last command in task '{task['name']}': {last_command} on device {last_device}")
                        # Store in strategy attributes
                        strategy.last_task_command = last_command
                        strategy.last_task_device = last_device
                
                # Extract and store global goal
                global_goal = task['global_goal']
                if global_goal:
                    strategy.global_pos_x = global_goal.get('x')
                    strategy.global_pos_y = global_goal.get('y')
                    # Support both 'angle' and 'theta' keys
                    strategy.global_pos_angle = global_goal.get('theta') 
                    strategy.get_logger().info(f"DEBUG: Raw global_goal dict = {global_goal}")
                    strategy.get_logger().info(f"DEBUG: After extraction - x={strategy.global_pos_x}, y={strategy.global_pos_y}, angle={strategy.global_pos_angle}")

                    if strategy.global_pos_x != None and strategy.global_pos_y != None and strategy.global_pos_angle != None :
                        strategy.get_logger().info(f"x= {strategy.global_pos_x} ")
                        strategy.get_logger().info(f"y= {strategy.global_pos_y} ")
                        strategy.get_logger().info(f"angle= {strategy.global_pos_angle} ")
                        strategy.get_logger().info(f"Global goal found for task '{task['name']}'")
                        strategy.handle_global_goal(float(strategy.global_pos_x)/1000, float(strategy.global_pos_y)/1000, float(strategy.global_pos_angle))
                        strategy.get_logger().info("global pose set goal")
                        strategy.global_pos_x =float( strategy.global_pos_x)
                        strategy.global_pos_y =float( strategy.global_pos_y)
                        strategy.global_pos_angle =float( strategy.global_pos_angle)
                        #strategy.pathfinder.set_goal(float(strategy.global_pos_x)/1000, float(strategy.global_pos_y)/1000,float(strategy.global_pos_angle))
                else:
                    strategy.get_logger().warning(f"⚠️  No global goal found for task: {task['name']}")
                
                # Load task into queues
                strategy.get_logger().warning(f"  loading  tasks {task['name']} into queues...")
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