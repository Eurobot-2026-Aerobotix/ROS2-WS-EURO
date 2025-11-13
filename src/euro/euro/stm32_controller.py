#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from service.srv import NaviStm
import time
class STM32Controller(Node):
    """
    ROS2 Node for controlling STM32 micro-ROS robot
    Provides high-level functions for robot control
    """
    
    def __init__(self, node_name='robot_stm32'):
        super().__init__(node_name)
        
        # Service client for STM32 navigation
        self.navi_client = self.create_client(
            NaviStm, 
            '/stm/navi_stm'  # Service name to be updated later
        )
        
        # Variables to store last robot_locate request (if needed)
        self.last_request = None
        self.get_logger().info('STM32 Controller initialized')

    def move_forward(self, distance, speed, timeout=10.0, wait_completion=True):
        """Send a request to move forward a certain distance at a specific speed"""
        if not self.navi_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('NaviStm service not available')
            return False, 'NaviStm service not available'
        
        request = NaviStm.Request()
        request.arg1 = 1.0  # Indicates move_forward command
        request.arg2 = distance
        request.arg3 = speed
        request.arg4 = 0.0
        request.arg5 = 0.0
        request.arg6 = 0.0
        request.arg7 = 0.0
        request.arg8 = 0.0
        request.arg9 = 0.0
        request.arg10 = 0.0
        self.get_logger().info(f"Sending move_forward: Distance={distance}, Speed={speed}")
        
        try:
            future = self.navi_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
            else:
                self.get_logger().error('Move_forward service call timeout')
                return False, 'Service call timeout'
        except Exception as e:
            self.get_logger().error(f'Move_forward service error: {str(e)}')
            return False, f'Service error: {str(e)}'
    
    def move_backward(self, distance, speed, timeout=10.0, wait_completion=True):
        """Send a request to move backward a certain distance at a specific speed"""
        if not self.navi_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('NaviStm service not available')
            return False, 'NaviStm service not available'
        
        request = NaviStm.Request()
        request.arg1 = 2.0  # Indicates move_backward command
        request.arg2 = distance
        request.arg3 = speed
        request.arg4 = 0.0
        request.arg5 = 0.0
        request.arg6 = 0.0
        request.arg7 = 0.0
        request.arg8 = 0.0
        request.arg9 = 0.0
        request.arg10 = 0.0
        self.get_logger().info(f"Sending move_backward: Distance={distance}, Speed={speed}")
        
        try:
            future = self.navi_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
            else:
                self.get_logger().error('Move_backward service call timeout')
                return False, 'Service call timeout'
        except Exception as e:
            self.get_logger().error(f'Move_backward service error: {str(e)}')
            return False, f'Service error: {str(e)}'
    
    def rotate(self, angle, is_radian, timeout=10.0, wait_completion=True):
        """Send a request to rotate by a certain angle"""
        if not self.navi_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('NaviStm service not available')
            return False, 'NaviStm service not available'
        
        request = NaviStm.Request()
        request.arg1 = 3.0  # Indicates rotate command
        request.arg2 = angle
        request.arg3 = is_radian
        request.arg4 = 0.0
        request.arg5 = 0.0
        request.arg6 = 0.0
        request.arg7 = 0.0
        request.arg8 = 0.0
        request.arg9 = 0.0
        request.arg10 = 0.0
        self.get_logger().info(f"Sending rotate: Angle={angle}, IsRadian={is_radian}")
        
        try:
            future = self.navi_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if future.done():
                response = future.result()
            else:
                self.get_logger().error('Rotate service call timeout')
                return False, 'Service call timeout'
        except Exception as e:
            self.get_logger().error(f'Rotate service error: {str(e)}')
            return False, f'Service error: {str(e)}'
    
    def orientate(self, orientation, is_radian, timeout=10.0, wait_completion=True):
        """Send a request to orientate to a specific orientation"""
        if not self.navi_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('NaviStm service not available')
            return False, 'NaviStm service not available'
        
        request = NaviStm.Request()
        request.arg1 = 4.0  # Indicates orientate command
        request.arg2 = orientation
        request.arg3 = is_radian
        request.arg4 = 0.0
        request.arg5 = 0.0
        request.arg6 = 0.0
        request.arg7 = 0.0
        request.arg8 = 0.0
        request.arg9 = 0.0
        request.arg10 = 0.0
        self.get_logger().info(f"Sending orientate: Orientation={orientation}, IsRadian={is_radian}")
        
        try:
            # Phase 1: Send command and wait for acceptance
            future = self.navi_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
            else:
                self.get_logger().error('Orientate service call timeout')
                return False, 'Service call timeout'
        except Exception as e:
            self.get_logger().error(f'Orientate service error: {str(e)}')
            return False, f'Service error: {str(e)}'
    
    def go_to(self, x, y, speed, timeout=10.0, wait_completion=True):
        """Send a request to go to a specific position (x, y) at given speed"""
        if not self.navi_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('NaviStm service not available')
            return False, 'NaviStm service not available'
        
        request = NaviStm.Request()
        request.arg1 = 5.0  # Indicates go_to command
        request.arg2 = x
        request.arg3 = y
        request.arg4 = speed
        request.arg5 = 0.0
        request.arg6 = 0.0
        request.arg7 = 0.0
        request.arg8 = 0.0
        request.arg9 = 0.0
        request.arg10 = 0.0
        self.get_logger().info(f"Sending go_to: X={x}, Y={y}, Speed={speed}")
        
        try:
            future = self.navi_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
            else:
                self.get_logger().error('Go_to service call timeout')
                return False, 'Service call timeout'
        except Exception as e:
            self.get_logger().error(f'Go_to service error: {str(e)}')
            return False, f'Service error: {str(e)}'
    def move_forward_with_left_deviation(self, distance, deviation, speed, timeout=10.0, wait_completion=True):
        """Send a request to move forward with left deviation"""
        if not self.navi_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('NaviStm service not available')
            return False, 'NaviStm service not available'
        
        request = NaviStm.Request()
        request.arg1 = 8.0  # Indicates move_forward_with_left_deviation command
        request.arg2 = distance
        request.arg3 = deviation
        request.arg4 = speed
        request.arg5 = 0.0
        request.arg6 = 0.0
        request.arg7 = 0.0
        request.arg8 = 0.0
        request.arg9 = 0.0
        request.arg10 = 0.0
        self.get_logger().info(f"Sending move_forward_with_left_deviation: Distance={distance}, Deviation={deviation}, Speed={speed}")
        
        try:
            future = self.navi_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
            else:
                self.get_logger().error('Move_forward_with_left_deviation service call timeout')
                return False, 'Service call timeout'
        except Exception as e:
            self.get_logger().error(f'Move_forward_with_left_deviation service error: {str(e)}')
            return False, f'Service error: {str(e)}'
    def circle_curve(self, x, y, angle,speed, timeout=10.0, wait_completion=True):
        """Send a request to rotate by a certain angle"""
        if not self.navi_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('NaviStm service not available')
            return False, 'NaviStm service not available'
        
        request = NaviStm.Request()
        request.arg1 = 9.0  # Indicates circle_curve command
        request.arg2 = x
        request.arg3 = y
        request.arg4 = angle
        request.arg5 = speed
        request.arg6 = 0.0
        request.arg7 = 0.0
        request.arg8 = 0.0
        request.arg9 = 0.0
        request.arg10 = 0.0
        self.get_logger().info(f"Sending circle curve: X={x}, Y={y}, Angle={angle}, Speed={speed}")
        
        try:
            future = self.navi_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
            else:
                self.get_logger().error('circle curve  service call timeout')
                return False, 'Service call timeout'
        except Exception as e:
            self.get_logger().error(f'circle curve  service error: {str(e)}')
            return False, f'Service error: {str(e)}'
    def led_blink(self, timeout=10.0):
        """Send a request to blink LED (synchronous)"""
        if not self.navi_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('NaviStm service not available')
            return False, 'NaviStm service not available'
            
        request = NaviStm.Request()
        request.arg1 = 0.0  # Indicates LED blink command
        request.arg2 = 0.0
        request.arg3 = 0.0
        request.arg4 = 0.0
        request.arg5 = 0.0
        request.arg6 = 0.0
        request.arg7 = 0.0
        request.arg8 = 0.0
        request.arg9 = 0.0
        request.arg10 = 0.0
        self.get_logger().info(f"Envoi de led blink")
        
        try:
            future = self.navi_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
                self.get_logger().info(f"LED blink response: {response.result}")
                return True, response
            else:
                self.get_logger().error('LED blink service call timeout')
                return False, 'Service call timeout'
        except Exception as e:
            self.get_logger().error(f'LED blink service error: {str(e)}')
            return False, f'Service error: {str(e)}'

    def IDLE(self, timeout=10.0):
        """
        Check if the current navigation command has completed
        
        Returns:
            tuple: (success, is_completed, response)
                   success: bool - if the service call was successful
                   is_completed: bool - if the movement command has finished
                   response: service response or error message
        """
        if not self.navi_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('NaviStm service not available for status check')
            return False, False, 'NaviStm service not available'
        
        request = NaviStm.Request()
        request.arg1 = 11.0  # Indicates status command
        request.arg2 = 0.0
        request.arg3 = 0.0
        request.arg4 = 0.0
        request.arg5 = 0.0
        request.arg6 = 0.0
        request.arg7 = 0.0
        request.arg8 = 0.0
        request.arg9 = 0.0
        request.arg10 = 0.0
        
        try:
            future = self.navi_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
                # Assuming response.result is True when movement is completed, False when still moving
                is_completed = bool(response.result)
                return True, is_completed, response
            else:
                self.get_logger().error('Status check timeout')
                return False, False, 'Status check timeout'
        except Exception as e:
            self.get_logger().error(f'Status check error: {str(e)}')
            return False, False, f'Status check error: {str(e)}'
    def wait_for_completion(self, max_wait_time=30.0, check_interval=0.1):
        """
        Wait for the current navigation command to complete using continuous status polling
        
        Args:
            max_wait_time (float): Maximum time to wait in seconds
            check_interval (float): Time between status checks in seconds
        
        Returns:
            tuple: (success, final_response)
        """
        start_time = time.time()
        check_count = 0
        
        self.get_logger().info("Waiting for movement to complete...")
        
        while rclpy.ok():
            # Check if we've exceeded maximum wait time
            elapsed_time = time.time() - start_time
            if elapsed_time > max_wait_time:
                self.get_logger().warning(f"Movement timeout after {max_wait_time} seconds")
                return False, "Movement timeout"
            
            # Check status
            success, is_completed, response = self.check_status()
            check_count += 1
            #emergency break in 6 checks (0.6s)
            # if(check_count == 6):
            #     self.stop()
            #     return False, "Emergency stop triggered"
            #     time.sleep(10)
            if not success:
                self.get_logger().error(f"Failed to check movement status (attempt {check_count})")
                return False, response
            
            if is_completed:
                self.get_logger().info(f"Movement completed successfully! (checked {check_count} times)")
                return True, response
            
            # Log progress every second (10 checks at 0.1s interval)
            if check_count % 10 == 0:
                self.get_logger().info(f"Still waiting for movement completion... (elapsed: {elapsed_time:.1f}s)")
            
            # Wait before next check
            time.sleep(check_interval)
        
        return False, "ROS shutdown during wait"

    def stop(self, timeout=5.0):
        """Send an emergency stop command to the STM32"""
        if not self.navi_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('NaviStm service not available for stop command')
            return False, 'NaviStm service not available'
        
        request = NaviStm.Request()
        request.arg1 = 10.0  # Indicates stop command (adjust this value based on your STM32 implementation)
        request.arg2 = 0.0
        request.arg3 = 0.0
        request.arg4 = 0.0
        request.arg5 = 0.0
        request.arg6 = 0.0
        request.arg7 = 0.0
        request.arg8 = 0.0
        request.arg9 = 0.0
        request.arg10 = 0.0
        self.get_logger().info("Sending EMERGENCY STOP command")
        
        try:
            future = self.navi_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
                self.get_logger().info(f"Emergency stop response: {response.result}")
                return True, response
            else:
                self.get_logger().error('Stop command timeout')
                return False, 'Stop timeout'
        except Exception as e:
            self.get_logger().error(f'Stop command error: {str(e)}')
            return False, f'Stop error: {str(e)}'

    def send_navi_command(self, request_msg, timeout=10.0):
        """
        Send navigation command to STM32 via NaviStm service
        
        Args:
            request_msg (NaviStm.Request): Request message for NaviStm service
            timeout (float): Service call timeout in seconds
        Returns:
            tuple: (success, response or error message)
        """
        if not self.navi_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('NaviStm service not available')
            return False, 'NaviStm service not available'
        
        try:
            future = self.navi_client.call_async(request_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if future.result() is not None:
                response = future.result()
                self.get_logger().info(f'NaviStm service response: {response}')
                return True, response
            else:
                self.get_logger().error('NaviStm service call failed')
                return False, 'Service call failed'
        except Exception as e:
            self.get_logger().error(f'NaviStm service error: {str(e)}')
            return False, f'Service error: {str(e)}'
    def reset(self, timeout=10.0):
        """Send a reset command to the STM32 (synchronous)"""
        # Keep trying to connect to service until timeout
        if not self.navi_client.wait_for_service(timeout_sec=10):
            self.get_logger().error(f'NaviStm service not available after 10 seconds')
            return False, f'NaviStm service not available after 10 seconds'

        self.get_logger().info('NaviStm service is now available!')
        request = NaviStm.Request()
        request.arg1 = 6.0  # Indicates reset command
        request.arg2 = 0.0
        request.arg3 = 0.0
        request.arg4 = 0.0
        request.arg5 = 0.0
        request.arg6 = 0.0
        request.arg7 = 0.0
        request.arg8 = 0.0
        request.arg9 = 0.0
        request.arg10 = 0.0
        self.get_logger().info("Envoi de reset command")
        
        try:
            future = self.navi_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
                self.get_logger().info(f"Reset response: {response.result}")
                return True, response
            else:
                self.get_logger().error('Reset service call timeout')
                return False, 'Service call timeout'
        except Exception as e:
            self.get_logger().error(f'Reset service error: {str(e)}')
            return False, f'Service error: {str(e)}'
def main(args=None):
    rclpy.init(args=args)
    controller = STM32Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('STM32 Controller shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()