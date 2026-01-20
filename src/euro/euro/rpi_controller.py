#!/usr/bin/env python3
"""
Raspberry Pi Controller - GPIO button control for manual start
"""

import RPi.GPIO as GPIO
import time

class RPiController:
    """Controller for Raspberry Pi GPIO operations"""
    
    def __init__(self):
        """Initialize GPIO pins"""
        # Use BCM pin numbering
        GPIO.setmode(GPIO.BCM)
        
        # Disable warnings
        GPIO.setwarnings(False)
        
        # Pin 27: Set as OUTPUT and always HIGH
        self.pin_high = 27
        GPIO.setup(self.pin_high, GPIO.OUT)
        GPIO.output(self.pin_high, GPIO.HIGH)
        
        # Pin 17: Set as INPUT to read button state
        self.pin_read = 17
        GPIO.setup(self.pin_read, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    def wait_start(self):
        """
        Wait for button press on pin 17
        Pin 27 is kept HIGH, pin 17 reads the button state
        Returns the value of pin 17 when button is pressed
        """
        print("⏸️  Waiting for manual start button press (pin 17)...")
        
        # Wait until pin 17 goes HIGH (button pressed)
        while GPIO.input(self.pin_read) == GPIO.LOW:
            time.sleep(0.01)  # 10ms polling
        
        # Button is pressed, get the value
        value = GPIO.input(self.pin_read)
        print(f"✅ Button pressed! Pin 17 value: {value}")
        
        # Optional: Wait for button release to avoid multiple triggers
        while GPIO.input(self.pin_read) == GPIO.HIGH:
            time.sleep(0.01)
        
        return value
    
    def read_pin_17(self):
        """Read the current state of pin 17"""
        return GPIO.input(self.pin_read)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        GPIO.cleanup()
    
    def __del__(self):
        """Destructor to ensure GPIO cleanup"""
        try:
            GPIO.cleanup()
        except:
            pass
