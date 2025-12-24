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
            print(f"✅ STM32 IDLE check success: {success}, is_completed: {is_completed}")
            if not success:
                strategy.get_logger().warning("⚠️ Failed to check robot status")
            else:
                IDLE = is_completed
        
        # Handle emergency stop
        if EMERGENCY_STOP:
            strategy.get_logger().warning("🚨 Emergency stop activated!")
            success, response = stm32.stop()
        
        time.sleep(0.1)  # 100ms


def stm32_task(strategy, stm32):
    """Runs every 100ms to process STM32 queue"""
    global IDLE
    
    while rclpy.ok():
        # Check if there are commands in the queue and robot is idle
        if len(strategy.queues['stm32']) > 0 and IDLE:
            cmd = strategy.queues['stm32'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            strategy.get_logger().info(f"📤 STM32: Executing {command} with {params}")
            
            # Execute command based on type
            if command == 'move_forward':
                stm32.move_forward(params['distance'], params['speed'])
                IDLE = False
            elif command == 'rotate':
                stm32.rotate(params['angle'], params['is_radian'])
                IDLE = False
            elif command == 'go_to':
                stm32.go_to(params['x'], params['y'], params['speed'])
                IDLE = False
            elif command == 'orientate':
                stm32.orientate(params['angle'], params['is_radian'])
                IDLE = False
            else:
                strategy.get_logger().warn(f"Unknown STM32 command: {command}")
            
            # Remove command from queue after execution
            strategy.queues['stm32'].popleft()
        
        time.sleep(0.1)  # 100ms


def esp1_task(strategy, esp1_controller=None):
    """Runs every 100ms to process ESP32_1 queue"""
    while rclpy.ok():
        # Check if there are commands in the queue
        if len(strategy.queues['esp32_1']) > 0:
            cmd = strategy.queues['esp32_1'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            strategy.get_logger().info(f"📤 ESP32_1: Executing {command} with {params}")
            
            # TODO: Add execution logic here
            # Example:
            # if command == 'change':
            #     esp1_controller.change(params)
            # elif command == 'request':
            #     esp1_controller.request(params)
            
            # Remove command from queue after execution
            strategy.queues['esp32_1'].popleft()
        
        time.sleep(0.1)  # 100ms


def esp2_task(strategy, esp2_controller=None):
    """Runs every 100ms to process ESP32_2 queue"""
    while rclpy.ok():
        # Check if there are commands in the queue
        if len(strategy.queues['esp32_2']) > 0:
            cmd = strategy.queues['esp32_2'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            strategy.get_logger().info(f"📤 ESP32_2: Executing {command} with {params}")
            
            # TODO: Add execution logic here
            # Example:
            # if command == 'move':
            #     esp2_controller.move(params)
            # elif command == 'calculate':
            #     esp2_controller.calculate(params)
            
            # Remove command from queue after execution
            strategy.queues['esp32_2'].popleft()
        
        time.sleep(0.1)  # 100ms


def esp3_task(strategy, esp3_controller=None):
    """Runs every 100ms to process ESP32_3 queue"""
    while rclpy.ok():
        # Check if there are commands in the queue
        if len(strategy.queues['esp32_3']) > 0:
            cmd = strategy.queues['esp32_3'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            strategy.get_logger().info(f"📤 ESP32_3: Executing {command} with {params}")
            
            # TODO: Add execution logic here
            # Example:
            # if command == 'led_pattern':
            #     esp3_controller.led_pattern(params['pattern'], params['color'])
            
            # Remove command from queue after execution
            strategy.queues['esp32_3'].popleft()
        
        time.sleep(0.1)  # 100ms
