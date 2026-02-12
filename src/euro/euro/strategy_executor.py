#!/usr/bin/env python3
"""
Strategy Executor - Device task executors for the strategy manager
Handles periodic execution of commands from device queues
"""

import rclpy
import time
from geometry_msgs.msg import Point

class Round:
    """Timer class to track game round time"""
    
    def __init__(self):
        self._start_time = None
    
    def start_game(self):
        """Start the game timer"""
        self._start_time = time.time()
    
    def time(self):
        """Return elapsed seconds since start_game() was called"""
        if self._start_time is None:
            return 0.0
        return time.time() - self._start_time

# Global state variables
IDLE = True
IDLE_FOR_SYSTEMS=True
PREVIOUS_IDLE_FOR_SYSTEMS=True
SYSTEMS_IDLE_COUNT=0
EMERGENCY_STOP = False
ESP_FRONT_IDLE=True
ESP_BACK_IDLE=True
color_pt_front=None
color_pt_back=None
color_pt=None
SHUTDOWN_REQUESTED = False  # Global shutdown flag
WAITING_TIRETTE=False
last_stm_command=""
def robot_status(strategy, stm32, lidar):
    """Runs every 100ms to check STM32 movement status and lidar obstacles"""
    global IDLE
    global EMERGENCY_STOP
    global SHUTDOWN_REQUESTED
    first_emergency = True
    global IDLE_FOR_SYSTEMS
    global PREVIOUS_IDLE_FOR_SYSTEMS
    global SYSTEMS_IDLE_COUNT
    global last_stm_command
    while rclpy.ok() and not SHUTDOWN_REQUESTED:        
        # Only check if robot is idle when we think it's busy
        if not IDLE:
            if not IDLE_FOR_SYSTEMS :
                success, is_completed, _ = stm32.IDLE_FOR_SYSTEMS(1.0)
                strategy.get_logger().info(f"idle for systems  : {is_completed}")
            else:
                success, is_completed, _ = stm32.IDLE(1.0)
                strategy.get_logger().info(f"idle : {is_completed}")
                
            if not success:
                strategy.get_logger().warning("⚠️ Failed to check robot status")
            else:
                if not IDLE_FOR_SYSTEMS and not EMERGENCY_STOP:
                    PREVIOUS_IDLE_FOR_SYSTEMS=IDLE_FOR_SYSTEMS
                    IDLE_FOR_SYSTEMS = is_completed
                else:
                    IDLE = is_completed
            if  IDLE_FOR_SYSTEMS ==True and PREVIOUS_IDLE_FOR_SYSTEMS==False and last_stm_command=="navigation":
                SYSTEMS_IDLE_COUNT+=1
            PREVIOUS_IDLE_FOR_SYSTEMS=IDLE_FOR_SYSTEMS
        
        time.sleep(0.1)  # 100ms

def lidar_status(strategy, stm32, lidar, round_timer):
    """Runs every 100ms to check lidar status and handle emergency stops"""
    global EMERGENCY_STOP
    global SHUTDOWN_REQUESTED
    first_emergency = True
    
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Update lidar with current round time
        lidar.set_round_time(round_timer.time())
        
        # Check emergency stop from lidar
        EMERGENCY_STOP = lidar.getstatus()
        
        # Handle emergency stop
        if EMERGENCY_STOP and first_emergency:
            strategy.get_logger().warning("🚨 Emergency stop activated!")
            #success, response = stm32.stop()
            first_emergency = False
        
        # Reset flag when emergency is cleared
        if not EMERGENCY_STOP:
            first_emergency = True
        
        time.sleep(0.01)  # 10ms

def to_float_list(params):
    """Convert all params to float"""
    try:
        return [float(p) for p in params]
    except (TypeError, ValueError) as e:
        raise ValueError(f"Invalid STM32 params: {params}") from e
def stm32_task(strategy, stm32,lidar):
    """Runs every 100ms to process STM32 queue"""
    global IDLE
    global SHUTDOWN_REQUESTED
    global IDLE_FOR_SYSTEMS
    global EMERGENCY_STOP
    global last_stm_command
    # Import Vector3 for publishing target position
    from geometry_msgs.msg import Vector3
    
    # Variables to save last command for resuming after emergency
    last_command = None
    last_params = None
    was_in_emergency = False
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue and robot is idle
        if len(strategy.queues['STM']) > 0 and IDLE:
            cmd = strategy.queues['STM'][0]  # Peek at first command
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            params = to_float_list(params)
            
            # Save command for potential resume after emergency
            last_command = command
            last_params = params
            last_stm_command=""
            # Execute command based on type
            if command == 'move_forward':
                stm32.move_forward(params[0], params[1], params[2] if len(params) > 2 else 0.0)
                IDLE = False
                IDLE_FOR_SYSTEMS=False
            elif command == 'rotate':
                stm32.rotate(params[0], params[1] if len(params) > 1 else 0.0)
                IDLE = False
                IDLE_FOR_SYSTEMS=False
            elif command == 'go_to':
                lidar.set_goto_target(params[0]/1000, params[1]/1000)  # Convert mm to meters for lidar
                IDLE_FOR_SYSTEMS=False
                
                if len(params) > 3:
                    stm32.go_to(params[0], params[1], params[2], params[3])
                else:
                    stm32.go_to(params[0], params[1], params[2])
                IDLE = False
                
            elif command == 'orientate':
                stm32.orientate(params[0], params[1] if len(params) > 1 else 0.0)
                IDLE = False
                IDLE_FOR_SYSTEMS=False
            elif command == 'bezier':
                if len(params) > 9:
                    stm32.bezier_curve(params[0], params[1], params[2], params[3], params[4], params[5], params[8], params[9])
                else:
                    stm32.bezier_curve(params[0], params[1], params[2], params[3], params[4], params[5], params[8])
                IDLE = False
                IDLE_FOR_SYSTEMS=False
            
            elif command == 'go_to_backwards':
                # Publish target position for collision detection
                lidar.set_goto_target(params[0]/1000, params[1]/1000)  # Convert mm to meters for lidar
                
                if len(params) > 3:
                    stm32.go_to_backwards(params[0], params[1], params[2], params[3])
                else:
                    stm32.go_to_backwards(params[0], params[1], params[2])
                IDLE = False
                IDLE_FOR_SYSTEMS=False
            elif command == 'bezier_backwards':
                if len(params) > 9:
                    stm32.bezier_curve_backwards(params[0], params[1], params[2], params[3], params[4], params[5], params[8], params[9])
                else:
                    stm32.bezier_curve_backwards(params[0], params[1], params[2], params[3], params[4], params[5], params[8])
                IDLE = False
                IDLE_FOR_SYSTEMS=False
            elif command == 'set_position':
                stm32.set_position(params[0], params[1], params[2])
            else:
                strategy.get_logger().warn(f"Unknown STM32 command: {command}")
            strategy.command_waiting=False
            if command !="set_position":
                last_stm_command="navigation"
            # Wait for command completion with emergency handling
            while (not IDLE and not SHUTDOWN_REQUESTED) or  (EMERGENCY_STOP) or last_stm_command=="emergency":
                # Check for emergency stop
               
                if EMERGENCY_STOP and not was_in_emergency:
                    strategy.get_logger().warning("🚨 Emergency detected during movement - stopping STM32")
                    stm32.stop()
                    last_stm_command="emergency"
                    IDLE=False
                    IDLE_FOR_SYSTEMS=False
                    was_in_emergency = True
                    while(not IDLE and not SHUTDOWN_REQUESTED):
                        strategy.get_logger().info(f"IDLE: {IDLE}")
                        time.sleep(0.1)  # Wait until STM32 reports idle after stop
                    lidar.start_waiting()  # Start waiting period before rechecking obstacle
                    #IDLE = True  # Mark as idle to prevent status checks
                
                # Check if emergency cleared
                elif not EMERGENCY_STOP and was_in_emergency :
                    strategy.get_logger().info("✅ Emergency cleared - resuming command")
                    # Resend the last command
                    #time.sleep(1)
                    if last_command == 'move_forward':
                        stm32.move_forward(last_params[0], last_params[1], last_params[2] if len(last_params) > 2 else 0.0)
                    elif last_command == 'rotate':
                        stm32.rotate(last_params[0], last_params[1] if len(last_params) > 1 else 0.0)
                    elif last_command == 'go_to':
                        if len(last_params) > 3:
                            stm32.go_to(last_params[0], last_params[1], last_params[2], last_params[3])
                        else:
                            stm32.go_to(last_params[0], last_params[1], last_params[2])
                    elif last_command == 'orientate':
                        stm32.orientate(last_params[0], last_params[1] if len(last_params) > 1 else 0.0)
                    elif last_command == 'bezier':
                        if len(last_params) > 9:
                            stm32.bezier_curve(last_params[0], last_params[1], last_params[2], last_params[3], last_params[4], last_params[5], last_params[8], last_params[9])
                        else:
                            stm32.bezier_curve(last_params[0], last_params[1], last_params[2], last_params[3], last_params[4], last_params[5], last_params[8])
                    elif last_command == 'go_to_backwards':
                        if len(last_params) > 3:
                            stm32.go_to_backwards(last_params[0], last_params[1], last_params[2], last_params[3])
                        else:
                            stm32.go_to_backwards(last_params[0], last_params[1], last_params[2])
                    elif last_command == 'bezier_backwards':
                        if len(last_params) > 9:
                            stm32.bezier_curve_backwards(last_params[0], last_params[1], last_params[2], last_params[3], last_params[4], last_params[5], last_params[8], last_params[9])
                        else:
                            stm32.bezier_curve_backwards(last_params[0], last_params[1], last_params[2], last_params[3], last_params[4], last_params[5], last_params[8])
                    last_stm_command="navigation"
                    was_in_emergency = False
                    IDLE = False  # Back to moving
                    IDLE_FOR_SYSTEMS=False

                time.sleep(0.1)
            strategy.get_logger().info(f"idle : {IDLE} :  EMERGENCY_STOP : {EMERGENCY_STOP}"  )
            strategy.get_logger().info( last_stm_command  )
            # Reset emergency flag when command completes normally
            was_in_emergency = False
            strategy.latest_robot_pose_beacon_count=0
            if command == 'bezier' or command == 'bezier_backwards':
                while( strategy.latest_robot_pose_beacon_count<2 and not SHUTDOWN_REQUESTED):
                    time.sleep(0.1)
                    strategy.get_logger().info(f"waitin two positions from nbeacon")
                stm_msg=0.0
                pos=None
                strategy.get_logger().info(f"len(strategy.latest_robot_pose_beacon.tracks):{len(strategy.latest_robot_pose_beacon.tracks)}")
                if strategy.latest_robot_pose_beacon and len(strategy.latest_robot_pose_beacon.tracks) > 0:
                    for track in strategy.latest_robot_pose_beacon.tracks:
                        if track.aruco_id==10:
                            stm_msg=1.0
                            pos=track.map_pos
                            strategy.get_logger().info(f"track.pos: {track.map_pos}")
                            break
                        elif track.aruco_id==6:
                            stm_msg=2.0
                            pos=track.map_pos
                            strategy.get_logger().info(f"track.pos: {track.map_pos}")
                            break
                if pos is not None:
                    strategy.get_logger().info(f"recevied pos from beacon : x{pos.x} , y : {pos.y}")
                    stm32.fix_position(stm_msg, pos.x, pos.y)
                else:
                    stm32.fix_position(stm_msg, params[6], params[7])
            # Remove command from queue after execution
            lidar.clear_goto_target()
            strategy.queues['STM'].popleft()
        
        time.sleep(0.1)  # 100ms
def handle_color_pattner(strategy):
    """Handle color pattern retrieval"""
    x = strategy.latest_robot_pose.x
    y = strategy.latest_robot_pose.y
    # Create Point for current position
    pos = Point()
    pos.x = float(x)
    pos.y = float(y)
    strategy.get_logger().info(f"robot position : x={x} , y={y}")
    pos.z = 0.0
    known_id = ''

    color_pattern, success, direction, batch_center = strategy.pathfinder.get_batch_color(known_id, pos)
    strategy.get_logger().info(f"pattern received fro beacon: {color_pattern} ")

    if success and batch_center:
        if (direction == "horizontal" and y < batch_center.y) or (direction == "vertical" and x > batch_center.x):
            # Reverse the color pattern
            color_pattern = color_pattern[::-1]
    else:
        strategy.get_logger().warning("Failed to get batch center from service")
    strategy.get_logger().info(f"color received from beacon{color_pattern}")

    return color_pattern
def convert_color_pattern(color_pattern, robot_color):
    """
    Convert color pattern from 'y'/'b' format to binary string.
    
    Args:
        color_pattern (str): Pattern string with 'y' (yellow) and 'b' (blue), e.g., "yybb", "ybyb"
        robot_color (str): The robot's color, either 'y' (yellow) or 'b' (blue)
    
    Returns:
        str: Binary string where robot's color (my color) is '0' and opposite color (not my color) is '1'
        
    Examples:
        convert_color_pattern("yybb", "y") -> "0011"  - first two are my color (0), last two are not (1)
        convert_color_pattern("yybb", "b") -> "1100" - first two are not my color (1), last two are (0)
        convert_color_pattern("ybyb", "y") -> "0101"  - y=0, b=1, y=0, b=1
        convert_color_pattern("ybyb", "b") -> "1010" - y=1, b=0, y=1, b=0
    """
    if not color_pattern:
        return "0"
    
    binary_str = ""
    for char in color_pattern.lower():
        if char == robot_color.lower():
            binary_str += "0"  # My color = 0
        else:
            binary_str += "1"  # Not my color = 1
    
    return binary_str


def stana(controller):
    """Wait for controller to be idle, with shutdown check"""
    global SHUTDOWN_REQUESTED
    
    # Use shorter timeout to allow quicker shutdown response
    idle = controller.IDLE(timeout=2.0)
    while not idle and not SHUTDOWN_REQUESTED:
        idle = controller.IDLE(timeout=2.0)
        time.sleep(0.1)
    
def talaa(strategy, front_controller=None):
    """Request color pattern from pathfinding service"""
    x = strategy.global_pos_x
    y = strategy.global_pos_y
    global color_pt
    strategy.get_logger().info(f"📞 talaa() called - about to request color pattern from pathfinder")
    # Create Point for current position
    # pos = Point()
    # pos.x = float(x)
    # pos.y = float(y)
    # pos.z = 0.0
    
    known_id = ''
    color_pattern=handle_color_pattner(strategy)
    success=True
    if success:
        # Convert global color ('yellow' or 'blue') to single letter ('y' or 'b')
        # Access color from strategy object instead of global variable
        if  strategy.color:
            robot_color = 'y' if strategy.color.lower() == 'yellow' else 'b'
        else:
            robot_color = 'y'  # Default to 'y' if color not set
        color_pt = convert_color_pattern(color_pattern, robot_color)
        color_to_N20=int(str(color_pt).replace('0', '3').replace('1', '4'))
        print(f"Converted color pattern: {color_pt}, N20 command: {color_to_N20}")
        front_controller.n20(color_to_N20)
        return color_pt
    else:
        strategy.get_logger().warning("⚠️ Failed to get color pattern")
        return None

#1 louta wl 2 fel wost wl 3 lfouk
def pick_up(controller,pneumatic,strategy,front):
    global color_pt_front
    global color_pt_back
    global ESP_FRONT_IDLE
    global ESP_BACK_IDLE
    global SHUTDOWN_REQUESTED
    
    strategy.get_logger().info(f"🚀 pick_up() called for {'FRONT' if front else 'BACK'}")
    
    if SHUTDOWN_REQUESTED:
        strategy.get_logger().warning("⚠️ pick_up() aborted - SHUTDOWN_REQUESTED")
        return
    #stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    
    if front:
        pneumatic.front(0)
    else: pneumatic.back(0)
    controller.kabech(1)
    time.sleep(0.7)
    controller.n20(1111)
    stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    
    time.sleep(1)
    strategy.get_logger().info(f"🔍 Requesting NEW color pattern for {'FRONT' if front else 'BACK'} at position ({strategy.latest_robot_pose.x:.2f}, {strategy.latest_robot_pose.y:.2f})")
    color_pattern = talaa(strategy, controller)
    if front:
        color_pt_front = color_pattern
        ESP_FRONT_IDLE=True
        strategy.get_logger().info(f"📦 Front picked up - NEW Color pattern: {color_pt_front}")
    else:
        color_pt_back = color_pattern
        ESP_BACK_IDLE=True
        strategy.get_logger().info(f"📦 Back picked up - NEW Color pattern: {color_pt_back}")
    stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    controller.kabech(0)
    time.sleep(1)
def put_down(controller,pneumatic,strategy,front):
    global ESP_FRONT_IDLE
    global ESP_BACK_IDLE
    global color_pt_front
    global color_pt_back
    global SHUTDOWN_REQUESTED
    
    if SHUTDOWN_REQUESTED:
        return
    
    time.sleep(1)
    color_pattern = color_pt_front if front else color_pt_back
    strategy.get_logger().info(f"color_pattern {color_pattern}")
    color_to_N20=int(str(color_pattern).replace('1', '4').replace('0','1'))
    strategy.get_logger().info(f"TO n20 command  {color_to_N20}")
    #controller.n20(color_to_N20)
    #stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    
    if front:
        pneumatic.front(15)
    else: pneumatic.back(15)
    if front:
        ESP_FRONT_IDLE=True
    else: ESP_BACK_IDLE=True
    
    if SHUTDOWN_REQUESTED:
        return
    
    controller.n20(0)
    
    stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    
    if front:
        pneumatic.front(16)
    else: pneumatic.back(16)
def front_task(strategy, front_controller=None,pneumatic_controller=None):
    global color_pt_front
    global SHUTDOWN_REQUESTED
    """Runs every 100ms to process ESP32_1 queue"""
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue
        if len(strategy.queues['ESP_front']) > 0:
            cmd = strategy.queues['ESP_front'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            if command == 'prepare':
                action = params.get('action')
                if action == 'pick_up':
                    front_controller.n20(2222)
                    front_controller.dawer(0)
                    strategy.get_logger().info(f"🔧 Front prepare for pick_up: n20(2222)")
                else:
                    cmd_to_n20=int(str(color_pt_front).replace('0','2').replace('1','4'))
                    front_controller.dawer(1)
                    front_controller.n20(cmd_to_n20)
            elif command =='put_down':
                put_down(front_controller,pneumatic_controller,strategy,True)
            elif command =='pick_up':
                pick_up(front_controller,pneumatic_controller,strategy,True)
            strategy.queues['ESP_front'].popleft()
        
        time.sleep(0.1)  # 100ms


def back_task(strategy, back_controller=None,pneumatic_controller=None):
    """Runs every 100ms to process ESP32_1 queue"""
    global color_pt_back
    global SHUTDOWN_REQUESTED
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue
        if len(strategy.queues['ESP_back']) > 0:
            cmd = strategy.queues['ESP_back'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            if command == 'prepare':
                action = params.get('action')
                if action == 'pick_up':
                    back_controller.n20(2222)
                    back_controller.dawer(0)
                    strategy.get_logger().info(f"🔧 Back prepare for pick_up: n20(2222)")
                else:
                    cmd_to_n20=int(str(color_pt_back).replace('0','2').replace('1','4'))
                    back_controller.dawer(1)
                    back_controller.n20(cmd_to_n20)
            elif command =='put_down':
                put_down(back_controller,pneumatic_controller,strategy,False)
            elif command =='pick_up':
                pick_up(back_controller,pneumatic_controller,strategy,False)
               
            strategy.queues['ESP_back'].popleft()
        
        time.sleep(0.1)  # 100ms
def rasp_pi_task(strategy, round_timer, rpi_controller=None, stm32_controller=None, front_controller=None, back_controller=None, pneumatic_controller=None):
    """Runs every 100ms to process RASP_PI queue"""
    global SHUTDOWN_REQUESTED
    global WAITING_TIRETTE
    global IDLE
    global IDLE_FOR_SYSTEMS
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue
        if len(strategy.queues['RASP_PI']) > 0:
            cmd = strategy.queues['RASP_PI'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', [])
            wait = cmd.get('wait', False)
            
            # Execute RASP_PI commands
            if command == 'Wait for manual start':
                WAITING_TIRETTE=True
                if rpi_controller:
                    result = rpi_controller.wait_start()
                    stm32_controller.reset()
                    WAITING_TIRETTE=False
                    strategy.get_logger().info(f"✅ Tirette tjebdet  STRATEGY GOOOOOOOOO !!!!")
                    round_timer.start_game()
                    strategy.get_logger().info(f"⏱️  Game timer started!")
                else:
                    strategy.get_logger().warning("RPI controller not available - skipping wait_start")
            elif command == 'RESET':
                # Execute reset in background
                #if stm32_controller:
                    #stm32_controller.reset(timeout=2.0)
                if pneumatic_controller:
                    pneumatic_controller.front(16)
                    pneumatic_controller.back(16)
                if front_controller:
                    front_controller.kabech(0)
                    front_controller.dawer(0)
                    front_controller.n20(0)
                    stana(front_controller)
                if back_controller:
                    back_controller.kabech(0)
                    back_controller.dawer(0)
                    back_controller.n20(0)
                    stana(back_controller)
                # Allow time for messages to be sent
                time.sleep(0.1)
            elif command == 'set_position':
                strategy.get_logger().info("set position sent from the rasp queue")
                params = to_float_list(params)
                stm32_controller.set_position(params[0], params[1], params[2])
                IDLE_FOR_SYSTEMS=True
                IDLE=False

            else:
                strategy.get_logger().warn(f"Unknown RASP_PI command: {command}")
            
            # Remove command from queue after execution
            strategy.queues['RASP_PI'].popleft()
        
        time.sleep(0.1)  # 100ms
