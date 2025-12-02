#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from euro.stm32_controller import STM32Controller
from euro.lidar import Lidar
import time
import threading
IDLE=True
EMERGENCY_STOP=False
def robot_status(stm32,lidar):
    """Runs every 100 ms to check STM32 movement status."""
    global IDLE
    global EMERGENCY_STOP
    

    while rclpy.ok():
        # Call STM32Controller.IDLE() to check motion status
        EMERGENCY_STOP = lidar.getstatus()
        print(EMERGENCY_STOP)
        if not IDLE:
            success, is_completed, _ = stm32.IDLE()
            #print(f"✅ STM32 IDLE check success: {success}, is_completed: {is_completed}")
            if not success:
                stm32.get_logger().warning("⚠️ Failed to check robot status")
            else:
                IDLE = is_completed  # True if robot finished moving
                #print(f"🤖 Robot state: {'IDLE' if IDLE else 'MOVING'}")

        #stm32.get_logger().info(f"🤖 Robot state: {'IDLE' if IDLE else 'MOVING'}")
        if EMERGENCY_STOP:
            stm32.get_logger().warning("🚨 Emergency stop activated!")
            success,response=stm32.stop()
        time.sleep(0.1)  # every 100 ms

def main(args=None):
    rclpy.init(args=args)
    stm32 = STM32Controller('rpi_stm32')
    lidar = Lidar()
    global IDLE
    
    # Create a MultiThreadedExecutor to spin both nodes
    executor = MultiThreadedExecutor()
    executor.add_node(stm32)
    executor.add_node(lidar)
    
    status_thread = threading.Thread(target=robot_status, args=(stm32,lidar), daemon=True)
    status_thread.start()
    stm32.move_forward(1500.0,200.0)
    try:
        executor.spin()
    except KeyboardInterrupt:
        stm32.get_logger().info('Robot Strategy shutting down...')
    finally:
        # Cleanup all controllers
        stm32.destroy_node()
        lidar.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()