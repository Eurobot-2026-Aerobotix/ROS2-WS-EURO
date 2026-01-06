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
EMERGENCY_STOP = False

color_pt=None
def robot_status(strategy, stm32, lidar):
    """Runs every 100ms to check STM32 movement status and lidar obstacles"""
    global IDLE
    global EMERGENCY_STOP
    
    while rclpy.ok():
        # Check emergency stop from lidar
        # EMERGENCY_STOP = lidar.getstatus()
        
        # Check if robot is idle
        if not IDLE:
            success, is_completed, _ = stm32.IDLE()
            if not EMERGENCY_STOP:
                first_emergency=True
            print(f"✅ STM32 IDLE check success: {success}, is_completed: {is_completed}")
            if not success:
                strategy.get_logger().warning("⚠️ Failed to check robot status")
            else:
                IDLE = is_completed
        
        # Handle emergency stop
        if EMERGENCY_STOP and first_emergency:
            strategy.get_logger().warning("🚨 Emergency stop activated!")
            success, response = stm32.stop()
            first_emergency=False
        
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
    
    while rclpy.ok():
        # Check if there are commands in the queue and robot is idle
        if len(strategy.queues['STM']) > 0 and IDLE:
            cmd = strategy.queues['STM'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            params = to_float_list(params)
            strategy.get_logger().info(f"📤 STM32: Executing {command} with {params}")
            
            # Execute command based on type
            if command == 'move_forward':
                stm32.move_forward(params[0], params[1], params[2] if len(params) > 2 else 0.0)
                IDLE = False
            elif command == 'rotate':
                stm32.rotate(params[0], params[1] if len(params) > 1 else 0.0)
                IDLE = False
            elif command == 'go_to':
                stm32.go_to(params[0], params[1], params[2])
                IDLE = False
            elif command == 'orientate':
                stm32.orientate(params[0], params[1] if len(params) > 1 else 0.0)
                IDLE = False
            elif command == 'bezier':
                stm32.bezier_curve(params[0], params[1], params[2], params[3], params[4], params[5], params[6])
                IDLE = False
            elif command == 'go_to_backwards':
                stm32.go_to_backwards(params[0], params[1], params[2])
                IDLE = False
            elif command == 'bezier_backwards':
                stm32.bezier_curve_backwards(params[0], params[1], params[2], params[3], params[4], params[5], params[6])
                IDLE = False
            elif command == 'set_position':
                stm32.set_position(params[0], params[1], params[2])
            else:
                strategy.get_logger().warn(f"Unknown STM32 command: {command}")
            
            
            # Remove command from queue after execution
            strategy.queues['STM'].popleft()
        
        time.sleep(0.1)  # 100ms

def convert_color_pattern(color_pattern, robot_color):
    """
    Convert color pattern from 'y'/'b' format to decimal integer.
    
    Args:
        color_pattern (str): Pattern string with 'y' (yellow) and 'b' (blue), e.g., "yybb", "ybyb"
        robot_color (str): The robot's color, either 'y' (yellow) or 'b' (blue)
    
    Returns:
        int: Decimal value where robot's color is '1' and opposite color is '0'
        
    Examples:
        convert_color_pattern("yybb", "y") -> 12 (binary 1100)
        convert_color_pattern("yybb", "b") -> 3 (binary 0011)
        convert_color_pattern("ybyb", "y") -> 10 (binary 1010)
        convert_color_pattern("ybyb", "b") -> 5 (binary 0101)
    """
    if not color_pattern:
        return 0
    
    binary_str = ""
    for char in color_pattern.lower():
        if char == robot_color.lower():
            binary_str += "0"
        else:
            binary_str += "1"
    
    return int(binary_str, 2)

def talaa(strategy, front_controller=None):
    """Request color pattern from pathfinding service"""
    x = strategy.global_pos_x
    y = strategy.global_pos_y
    
    # Create Point for current position
    # pos = Point()
    # pos.x = float(x)
    # pos.y = float(y)
    # pos.z = 0.0
    known_id = ''
    #color_pattern, success = strategy.pathfinder.get_batch_color(known_id, pos)
    color_pattern="yybb"
    success=True
    if success:
        color_pt=convert_color_pattern(color_pattern,"y")
        front_controller.n20(color_pt)
        return color_pattern
    else:
        strategy.get_logger().warning("⚠️ Failed to get color pattern")
        return None

def front_task(strategy, front_controller=None,pneumatic_controller=None):
    """Runs every 100ms to process ESP32_1 queue"""
    while rclpy.ok():
        # Check if there are commands in the queue
        if len(strategy.queues['ESP_front']) > 0:
            cmd = strategy.queues['ESP_front'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            strategy.get_logger().info(f"📤 Front: Executing {command} with {params}")
            if command =='put_down':
                pneumatic_controller.front(color_pt)
                time.sleep(0.3)
                front_controller.n20(16)
                front_controller.dawer(0)
                pneumatic_controller.front(15)
            if(command =='pick_up'):
                pneumatic_controller.front(0)
                front_controller.kabech(1)
                front_controller.n20(16)
                time.sleep(7)
                # idle=front_controller.IDLE()
                # while(not idle):
                #     idle=front_controller.IDLE()
                #     time.sleep(0.1)
                time.sleep(0.2)
                w=talaa(strategy,front_controller)
                strategy.get_logger().info(f"Color pattern received: {w}")
                time.sleep(7)
                # idle=front_controller.IDLE()
                # while(not idle):
                #     idle=front_controller.IDLE()
                #     time.sleep(0.1)
                front_controller.kabech(0)
                front_controller.dawer(1)
                
            
            # TODO: Add execution logic here
            # Example:
            # if command == 'change':
            #     front_controller.change(params)
            # elif command == 'request':
            #     front_controller.request(params)
            
            # Remove command from queue after execution
            strategy.queues['ESP_front'].popleft()
        
        time.sleep(0.1)  # 100ms


def back_task(strategy, back_controller=None):
    """Runs every 100ms to process ESP32_2 queue"""
    while rclpy.ok():
        # Check if there are commands in the queue
        if len(strategy.queues['ESP_back']) > 0:
            cmd = strategy.queues['ESP_back'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            strategy.get_logger().info(f"📤 back: Executing {command} with {params}")
            
            # TODO: Add execution logic here
            # Example:
            # if command == 'move':
            #     back_controller.move(params)
            # elif command == 'calculate':
            #     back_controller.calculate(params)
            
            # Remove command from queue after execution
            strategy.queues['ESP_back'].popleft()
        
        time.sleep(0.1)  # 100ms


def pneumatic_task(strategy, pneumatic_controller=None):
    """Runs every 100ms to process ESP32_3 queue"""
    while rclpy.ok():
        # Check if there are commands in the queue
        if len(strategy.queues['ESP_pneumatic']) > 0:
            cmd = strategy.queues['ESP_pneumatic'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            strategy.get_logger().info(f"📤 Pneumatic: Executing {command} with {params}")
            
            # TODO: Add execution logic here
            # Example:
            # if command == 'led_pattern':
            #     pneumatic_controller.led_pattern(params['pattern'], params['color'])
            
            # Remove command from queue after execution
            strategy.queues['ESP_pneumatic'].popleft()
        
        time.sleep(0.1)  # 100ms
