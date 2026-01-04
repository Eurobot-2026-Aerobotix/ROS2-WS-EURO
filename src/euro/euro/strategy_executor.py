#!/usr/bin/env python3
"""
Strategy Executor - Device task executors for the strategy manager
Handles periodic execution of commands from device queues
"""

import rclpy
import time

# Global state variables
IDLE = True
EMERGENCY_STOP = False


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


def front_task(strategy, front_controller=None):
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
                front_controller.n20(20)
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
