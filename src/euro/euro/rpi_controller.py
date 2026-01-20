#!/usr/bin/env python3
"""
Raspberry Pi Controller - GPIO button control for manual start
Gracefully handles non-RPi environments for testing
"""

import time
import rclpy
from rclpy.node import Node

# Try to import gpiozero, handle gracefully if not available or not on RPi
try:
    from gpiozero import DigitalOutputDevice, DigitalInputDevice, Device
    GPIO_AVAILABLE = True
    print("gpiozero library loaded successfully")
    # Set default pin factory to lgpio for better RPi 5 support
    try:
        from gpiozero.pins.lgpio import LGPIOFactory
        Device.pin_factory = LGPIOFactory()
        print("Using lgpio pin factory for RPi 5 compatibility")
    except ImportError:
        print("lgpio not available, using default pin factory")
except (ImportError, RuntimeError) as e:
    GPIO_AVAILABLE = False
    print(f"gpiozero not available: {e}")
    print("GPIO functions will be simulated for testing purposes")

class RPiController(Node):
    """ROS2 Node for Raspberry Pi GPIO operations"""
    
    def __init__(self, node_name='rpi_controller'):
        """Initialize GPIO pins"""
        super().__init__(node_name)
        
        # GPIO pin configuration
        self.VCC_PIN = 27        # Output pin for VCC (always HIGH)
        self.BUTTON_PIN = 17     # Input pin for button reading
        
        # GPIO device objects
        self.vcc_device = None
        self.button_device = None
        
        self.gpio_initialized = False
        
        if not GPIO_AVAILABLE:
            self.get_logger().warning("GPIO not available - running in simulation mode")
            return
        
        try:
            # Setup VCC pin as output (always HIGH)
            self.vcc_device = DigitalOutputDevice(self.VCC_PIN, initial_value=True)
            
            # Setup button pin as input with pull-down resistor
            self.button_device = DigitalInputDevice(self.BUTTON_PIN, pull_up=False)
            
            self.gpio_initialized = True
            self.get_logger().info("✅ GPIO pins initialized successfully using gpiozero")
            self.get_logger().info(f"VCC pin: GPIO {self.VCC_PIN} (Output, always HIGH)")
            self.get_logger().info(f"Button pin: GPIO {self.BUTTON_PIN} (Input with pulldown)")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize GPIO: {e}")
            self.gpio_initialized = False
    
    def wait_start(self):
        """
        Wait for button press on pin 17
        Pin 27 is kept HIGH, pin 17 reads the button state
        Returns the value of pin 17 when button is pressed
        """
        if not self.gpio_initialized:
            self.get_logger().warning("⚠️  GPIO not initialized - simulating button press after 1 second")
            time.sleep(1)
            self.get_logger().info("✅ Simulated button press!")
            return 1
        
        self.get_logger().info("⏸️  Waiting for manual start button press (pin 17)...")
        
        # Wait until pin 17 goes LOW (jbeda l tirtette)
        while  self.button_device.value:
            time.sleep(0.01)  # 10ms polling
        
        # Button is pressed, get the value
        value = self.button_device.value
        self.get_logger().info(f"✅ Triette  Removed ! Pin 17 value: {value}")
        
        return value
    
    def read_pin_17(self):
        """Read the current state of pin 17"""
        if not self.gpio_initialized:
            return 0
        return self.button_device.value
    
    def cleanup(self):
        """Clean up GPIO resources"""
        if self.gpio_initialized and GPIO_AVAILABLE:
            try:
                if self.vcc_device:
                    self.vcc_device.close()
                if self.button_device:
                    self.button_device.close()
                self.get_logger().info("GPIO cleanup completed")
            except Exception as e:
                self.get_logger().warning(f"GPIO cleanup warning: {e}")
    
    def __del__(self):
        """Destructor to ensure GPIO cleanup"""
        self.cleanup()

def main(args=None):
    """Main function for testing the GPIO controller"""
    rclpy.init(args=args)
    
    rpi_controller = RPiController()
    
    try:
        rpi_controller.get_logger().info('RPI Controller ready.')
        rclpy.spin(rpi_controller)
        
    except KeyboardInterrupt:
        rpi_controller.get_logger().info('RPI Controller shutting down...')
    finally:
        rpi_controller.cleanup()
        rpi_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
