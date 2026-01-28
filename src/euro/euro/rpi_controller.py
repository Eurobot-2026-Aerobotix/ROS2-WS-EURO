#!/usr/bin/env python3
"""n Raspberry Pi Controller - GPIO button control for manual start
Gracefully handles non-RPi environments for testing
"""

import time
import rclpy
from rclpy.node import Node
import sys
import threading

# Try to import gpiozero, handle gracefully if not available or not on RPi
try:
    from gpiozero import DigitalOutputDevice, DigitalInputDevice, Device
    GPIO_AVAILABLE = True
    # Set default pin factory to lgpio for better RPi 5 support
    try:
        from gpiozero.pins.lgpio import LGPIOFactory
        Device.pin_factory = LGPIOFactory()
    except (ImportError, Exception):
        pass  # Use default pin factory
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False

class RPiController(Node):
    """ROS2 Node for Raspberry Pi GPIO operations"""
    
    def __init__(self, node_name='rpi_controller'):
        """Initialize GPIO pins"""
        super().__init__(node_name)
        
        # GPIO pin configuration
        self.VCC_PIN = 27        # Output pin for VCC (always HIGH)
        self.BUTTON_PIN = 17     # Input pin for button reading
        self.STM32_RESET_PIN = 5 # Output pin for STM32 reset (active LOW)
        
        # GPIO device objects
        self.vcc_device = None
        self.button_device = None
        self.stm32_reset_device = None
        
        self.gpio_initialized = False
        
        if not GPIO_AVAILABLE:
            self.get_logger().warning("GPIO library not available - running in simulation mode")
            return
        
        try:
            # First, try to close any existing pin factory to release GPIO locks
            try:
                if Device.pin_factory is not None:
                    Device.pin_factory.close()
                    Device.pin_factory = None
            except Exception:
                pass
            
            # Re-initialize pin factory
            try:
                from gpiozero.pins.lgpio import LGPIOFactory
                Device.pin_factory = LGPIOFactory()
            except Exception:
                pass
            
            # Setup VCC pin as output (always HIGH)
            self.vcc_device = DigitalOutputDevice(self.VCC_PIN, initial_value=True)
            
            # Setup button pin as input with pull-down resistor
            self.button_device = DigitalInputDevice(self.BUTTON_PIN, pull_up=False)
            
            # Setup STM32 reset pin as output (initially HIGH - not in reset)
            self.stm32_reset_device = DigitalOutputDevice(self.STM32_RESET_PIN, initial_value=True)
            
            self.gpio_initialized = True
            self.get_logger().info(f"✅ GPIO initialized (VCC:{self.VCC_PIN}, Button:{self.BUTTON_PIN}, STM32_Reset:{self.STM32_RESET_PIN})")
        except PermissionError as pe:
            self.get_logger().error(f"❌ Permission denied: {pe}")
            self.get_logger().error("Try: sudo usermod -a -G gpio $USER")
            self.gpio_initialized = False
        except Exception as e:
            self.get_logger().error(f"❌ GPIO init failed: {e}")
            self.get_logger().error("Try: sudo pkill -f strategy")
            self.gpio_initialized = False
    
    def wait_start(self):
        """
        Wait for button press on pin 17
        Pin 27 is kept HIGH, pin 17 reads the button state
        Returns the value of pin 17 when button is pressed
        """
        if not self.gpio_initialized:
            self.get_logger().warning("GPIO not initialized - simulating button press")
            time.sleep(1)
            return 1
        
        self.get_logger().info("⏸️  Waiting for Triette ...")
        
        # Wait until pin 17 goes LOW (jbeda l tirtette)
        while self.button_device.value:
            time.sleep(0.01)  # 10ms polling
        
        value = self.button_device.value
        
        return value
    
    def read_pin_17(self):
        """Read the current state of pin 17"""
        if not self.gpio_initialized:
            return 0
        return self.button_device.value
    
    def write_stm32_reset_high(self):
        """
        Write HIGH to the STM32 reset pin (release reset)
        
        Returns:
            bool: True if write was successful, False otherwise
        """
        if not self.gpio_initialized:
            self.get_logger().warning("GPIO not initialized - cannot write to STM32 reset pin")
            return False
            
        try:
            self.stm32_reset_device.on()
            self.get_logger().info('STM32 reset pin set to HIGH (reset released)')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to write HIGH to STM32 reset pin: {str(e)}')
            return False
    
    def write_stm32_reset_low(self):
        """
        Write LOW to the STM32 reset pin (activate reset)
        
        Returns:
            bool: True if write was successful, False otherwise
        """
        if not self.gpio_initialized:
            self.get_logger().warning("GPIO not initialized - cannot write to STM32 reset pin")
            return False
            
        try:
            self.stm32_reset_device.off()
            self.get_logger().info('STM32 reset pin set to LOW (reset active)')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to write LOW to STM32 reset pin: {str(e)}')
            return False
    
    def reset_stm32(self, hold_time_ms=100):
        """
        Perform a complete reset sequence on STM32
        Pulls reset LOW, waits, then releases back to HIGH
        
        Args:
            hold_time_ms: Time to hold reset LOW in milliseconds (default 100ms)
        
        Returns:
            bool: True if reset sequence completed successfully, False otherwise
        """
        if not self.gpio_initialized:
            self.get_logger().warning("GPIO not initialized - cannot reset STM32")
            return False
        
        try:
            # Pull reset LOW (activate reset)
            if not self.write_stm32_reset_low():
                return False
            
            # Hold reset for specified time
            #time.sleep(hold_time_ms / 1000.0)
            
            # Release reset back to HIGH
            if not self.write_stm32_reset_high():
                return False
            
            self.get_logger().info(f'✅ STM32 reset sequence completed ({hold_time_ms}ms)')
            return True
            
        except Exception as e:
            self.get_logger().error(f'STM32 reset sequence failed: {str(e)}')
            return False
    
    def cleanup(self):
        """Clean up GPIO resources"""
        if GPIO_AVAILABLE:
            # Install custom excepthook to suppress GPIO callback thread errors during cleanup
            original_excepthook = threading.excepthook
            def suppress_gpio_errors(args):
                # Suppress lgpio errors from background callback threads
                if 'lgpio' not in str(args.exc_type) and 'NoneType' not in str(args.exc_value):
                    original_excepthook(args)
            threading.excepthook = suppress_gpio_errors
            
            try:
                # Disable callbacks first to prevent race conditions with cleanup
                if self.button_device:
                    try:
                        self.button_device.when_activated = None
                        self.button_device.when_deactivated = None
                    except Exception:
                        pass
                
                # Close devices (may trigger background thread errors)
                if self.vcc_device:
                    try:
                        self.vcc_device.close()
                    except Exception:
                        pass
                    self.vcc_device = None
                    
                if self.button_device:
                    try:
                        self.button_device.close()
                    except Exception:
                        pass
                    self.button_device = None
                
                if self.stm32_reset_device:
                    try:
                        self.stm32_reset_device.close()
                    except Exception:
                        pass
                    self.stm32_reset_device = None
                
                # Close the pin factory to fully release GPIO
                if Device.pin_factory is not None:
                    try:
                        Device.pin_factory.close()
                    except Exception:
                        pass
                    Device.pin_factory = None
                
                # Give background threads time to finish
                time.sleep(0.15)
                
                self.gpio_initialized = False
                if hasattr(self, 'get_logger'):
                    self.get_logger().info("GPIO cleanup completed")
            except Exception as e:
                if hasattr(self, 'get_logger'):
                    self.get_logger().warning(f"GPIO cleanup warning: {e}")
            finally:
                # Restore original excepthook
                threading.excepthook = original_excepthook
    
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
