#!/usr/bin/env python3
"""
Strategy Executor - Device task executors for the strategy manager
Handles periodic execution of commands from device queues
"""

import rclpy
import time
from geometry_msgs.msg import Point

# Global state variables
IDLE = True
IDLE_FOR_STSEMS=True
EMERGENCY_STOP = False
ESP_FRONT_IDLE=True
ESP_BACK_IDLE=True
color_pt_front=None
color_pt_back=None
color_pt=None
SHUTDOWN_REQUESTED = False  # Global shutdown flag

def robot_status(strategy, stm32, lidar):
    """Runs every 100ms to check STM32 movement status and lidar obstacles"""
    global IDLE
    global EMERGENCY_STOP
    global SHUTDOWN_REQUESTED
    first_emergency = True
    
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Check emergency stop from lidar
        # EMERGENCY_STOP = lidar.getstatus()
        
        # Only check if robot is idle when we think it's busy
        if not IDLE:
            if not IDLE_FOR_STSEMS:   
                success, is_completed, _ = stm32.IDLE_FOR_SYSTEMS()
            else:
                success, is_completed, _ = stm32.IDLE()
            if not EMERGENCY_STOP:
                first_emergency = True
                
            if not success:
                strategy.get_logger().warning("⚠️ Failed to check robot status")
            else:
                if not IDLE_FOR_STSEMS:
                    IDLE_FOR_STSEMS = is_completed
                else:
                    IDLE = is_completed
        
        # Handle emergency stop
        if EMERGENCY_STOP and first_emergency:
            strategy.get_logger().warning("🚨 Emergency stop activated!")
            success, response = stm32.stop()
            first_emergency = False
        
        time.sleep(0.1)  # 100ms

def to_float_list(params):
    """Convert all params to float"""
    try:
        return [float(p) for p in params]
    except (TypeError, ValueError) as e:
        raise ValueError(f"Invalid STM32 params: {params}") from e
def stm32_task(strategy, stm32):
    """Runs every 100ms to process STM32 queue"""
    global IDLE
    global SHUTDOWN_REQUESTED
    global IDLE_FOR_STSEMS
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue and robot is idle
        if len(strategy.queues['STM']) > 0 and IDLE:
            cmd = strategy.queues['STM'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            params = to_float_list(params)
            # Execute command based on type
            if command == 'move_forward':
                stm32.move_forward(params[0], params[1], params[2] if len(params) > 2 else 0.0)
                IDLE = False
                IDLE_FOR_STSEMS=False
            elif command == 'rotate':
                stm32.rotate(params[0], params[1] if len(params) > 1 else 0.0)
                IDLE = False
                IDLE_FOR_STSEMS=False
            elif command == 'go_to':
                if len(params) > 3:
                    stm32.go_to(params[0], params[1], params[2], params[3])
                else:
                    stm32.go_to(params[0], params[1], params[2])
                IDLE = False
                IDLE_FOR_STSEMS=False
            elif command == 'orientate':
                stm32.orientate(params[0], params[1] if len(params) > 1 else 0.0)
                IDLE = False
                IDLE_FOR_STSEMS=False
            elif command == 'bezier':
                stm32.bezier_curve(params[0], params[1], params[2], params[3], params[4], params[5], params[6])
                IDLE = False
                IDLE_FOR_STSEMS=False
            elif command == 'go_to_backwards':
                if len(params) > 3:
                    stm32.go_to_backwards(params[0], params[1], params[2], params[3])
                else:
                    stm32.go_to_backwards(params[0], params[1], params[2])
                IDLE = False
                IDLE_FOR_STSEMS=False
            elif command == 'bezier_backwards':
                stm32.bezier_curve_backwards(params[0], params[1], params[2], params[3], params[4], params[5], params[6])
                IDLE = False
                IDLE_FOR_STSEMS=False
            elif command == 'set_position':
                stm32.set_position(params[0], params[1], params[2])
            else:
                strategy.get_logger().warn(f"Unknown STM32 command: {command}")
            strategy.command_waiting=False
            while not IDLE and not SHUTDOWN_REQUESTED: 
                time.sleep(0.1)
            # Remove command from queue after execution
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
    pos.z = 0.0
    known_id = ''
    color_pattern, success, direction, batch_center = strategy.pathfinder.get_batch_color(known_id, pos)
    
    if success and batch_center:
        if (direction == "horizontal" and y < batch_center.y) or (direction == "vertical" and x > batch_center.x):
            # Reverse the color pattern
            color_pattern = color_pattern[::-1]
    else:
        strategy.get_logger().warning("Failed to get batch center from service")

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
    idle = controller.IDLE(timeout=1.0)
    while not idle and not SHUTDOWN_REQUESTED:
        idle = controller.IDLE(timeout=1.0)
        time.sleep(0.1)
    
def talaa(strategy, front_controller=None):
    """Request color pattern from pathfinding service"""
    x = strategy.global_pos_x
    y = strategy.global_pos_y
    global color_pt
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
        color_to_N20=int(str(color_pt).replace('0', '2').replace('1', '3'))
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
    
    if SHUTDOWN_REQUESTED:
        return
    
    controller.n20(2222)
    stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    
    if front:
        pneumatic.front(0)
    else: pneumatic.back(0)
    controller.kabech(1)
    time.sleep(0.3)
    controller.n20(1111)
    stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    
    time.sleep(1)
    color_pattern = talaa(strategy, controller)
    if front:
        color_pt_front = color_pattern
        strategy.get_logger().info(f"📦 Front picked up - Color pattern: {color_pt_front}")
    else:
        color_pt_back = color_pattern
        strategy.get_logger().info(f"📦 Back picked up - Color pattern: {color_pt_back}")
    stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    
    if front:
        ESP_FRONT_IDLE=True
    else: ESP_BACK_IDLE=True
    strategy.get_logger().info("System is IDLE ")
    controller.kabech(0)
    controller.dawer(1)
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
    color_to_N20=int(str(color_pt_front).replace('1', '3').replace('0','1'))
    if front:
        strategy.get_logger().info(f"📤 Front putting down - Color pattern: {color_pt_front}")
    else:
        strategy.get_logger().info(f"📤 Back putting down - Color pattern: {color_pt_back}")
    controller.n20(color_to_N20)
    stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    
    if front:
        pneumatic.front(15)
    else: pneumatic.back(15)
    controller.dawer(0)
    time.sleep(1)
    controller.n20(2222)
    stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    
    if front:
        ESP_FRONT_IDLE=True
    else: ESP_BACK_IDLE=True
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
            
            if command =='put_down':
                put_down(front_controller,pneumatic_controller,strategy,True)
            if(command =='pick_up'):
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
            
            if command =='put_down':
                put_down(back_controller,pneumatic_controller,strategy,False)
            if(command =='pick_up'):
                pick_up(back_controller,pneumatic_controller,strategy,False)
               
            strategy.queues['ESP_back'].popleft()
        
        time.sleep(0.1)  # 100ms
def rasp_pi_task(strategy, rpi_controller=None, stm32_controller=None, front_controller=None, back_controller=None, pneumatic_controller=None):
    """Runs every 100ms to process RASP_PI queue"""
    global SHUTDOWN_REQUESTED
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue
        if len(strategy.queues['RASP_PI']) > 0:
            cmd = strategy.queues['RASP_PI'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', [])
            wait = cmd.get('wait', False)
            
            # Execute RASP_PI commands
            if command == 'Wait for manual start':
                if rpi_controller:
                    result = rpi_controller.wait_start()
                    strategy.get_logger().info(f"✅ Tirette tjebdet  STRATEGY GOOOOOOOOO !!!!")
                else:
                    strategy.get_logger().warning("RPI controller not available - skipping wait_start")
            elif command == 'RESET':
                # Execute reset in background
                if stm32_controller:
                    stm32_controller.stop(timeout=2.0)
                    while not IDLE:
                        time.sleep(0.1)
                    stm32_controller.reset(timeout=2.0)
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
            else:
                strategy.get_logger().warn(f"Unknown RASP_PI command: {command}")
            
            # Remove command from queue after execution
            strategy.queues['RASP_PI'].popleft()
        
        time.sleep(0.1)  # 100ms
