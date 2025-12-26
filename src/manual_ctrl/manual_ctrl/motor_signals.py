#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from gpiozero import PWMOutputDevice
import time
import threading

# GPIO pins (BCM numbering)
ESC_GPIO_PIN = 17
SERVO_GPIO_PIN = 18

# Servo parameters
SERVO_CENTER_US = 1500
SERVO_RANGE_US = 500
MAX_STEERING_RAD = 0.52

# ESC parameters - BIDIRECTIONAL
ESC_REVERSE_MAX_US = 1300    # Max reverse (was 1000, now 1300 for slower)
ESC_NEUTRAL_US = 1500       # Neutral - NO MOVEMENT
ESC_FORWARD_MAX_US = 1700   # Max forward (was 2000, now 1700 for slower)
MAX_SPEED_MPS = 1.0         # Reduced max speed (was 3.0)

PWM_FREQUENCY = 50  # 50 Hz

class RCCarPWMDriver(Node):
    def __init__(self):
        super().__init__('rc_car_pwm_driver')
        
        try:
            # Create PWM devices using gpiozero
            self.esc_pwm = PWMOutputDevice(ESC_GPIO_PIN, frequency=PWM_FREQUENCY)
            self.servo_pwm = PWMOutputDevice(SERVO_GPIO_PIN, frequency=PWM_FREQUENCY)
            
            # Start with neutral signals
            self.esc_pwm.value = self._us_to_value(ESC_NEUTRAL_US)
            self.servo_pwm.value = self._us_to_value(SERVO_CENTER_US)
            
            self.get_logger().info("GPIO PWM initialized successfully with gpiozero")
            self.get_logger().info(f"ESC Range: {ESC_REVERSE_MAX_US}us (reverse) to {ESC_FORWARD_MAX_US}us (forward)")
            self.get_logger().info(f"Max Speed: {MAX_SPEED_MPS} m/s")
            
            # ESC calibration
            self.calibrating = True
            self.get_logger().info("Starting ESC calibration...")
            threading.Thread(target=self._calibrate_esc, daemon=True).start()
            
            # Subscribe to drive commands
            self.subscription = self.create_subscription(
                AckermannDriveStamped,
                'ackermann_cmd',
                self.drive_callback,
                10
            )
            
            self.get_logger().info("RC Car PWM Driver ready for BIDIRECTIONAL control!")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO: {e}")
            raise
    
    def _calibrate_esc(self):
        """Calibrate ESC - BIDIRECTIONAL"""
        try:
            self.get_logger().info("ESC Calibration Phase 1: Max Forward")
            self.esc_pwm.value = self._us_to_value(ESC_FORWARD_MAX_US)
            time.sleep(5)
            
            self.get_logger().info("ESC Calibration Phase 2: Max Reverse")
            self.esc_pwm.value = self._us_to_value(ESC_REVERSE_MAX_US)
            time.sleep(1)
            
            self.get_logger().info("ESC Calibration Phase 3: Neutral")
            self.esc_pwm.value = self._us_to_value(ESC_NEUTRAL_US)
            time.sleep(1)
            
            self.calibrating = False
            self.get_logger().info("✓ ESC calibration complete! Ready to move.")
            
        except Exception as e:
            self.get_logger().error(f"ESC calibration error: {e}")
            self.calibrating = False
    
    def _us_to_value(self, pulse_us):
        """Convert microseconds to gpiozero value (0.0-1.0)
        At 50Hz: 1000us=5%, 1500us=7.5%, 2000us=10%
        So: value = pulse_us / 20000
        """
        return pulse_us / 20000.0
    
    def map_range(self, value, in_min, in_max, out_min, out_max):
        """Map value from one range to another"""
        value = np.clip(value, in_min, in_max)
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def convert_speed_to_pwm(self, speed_mps):
        """Convert speed in m/s to PWM pulse width in microseconds
        BIDIRECTIONAL:
        - Positive speed (forward): 1500 → 1700 us
        - Negative speed (reverse): 1500 → 1300 us
        """
        speed_mps = np.clip(speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS)
        
        if speed_mps > 0:
            # Forward: 0 to MAX_SPEED_MPS → 1500 to 1700 us
            pwm = self.map_range(speed_mps, 0.0, MAX_SPEED_MPS, 
                                ESC_NEUTRAL_US, ESC_FORWARD_MAX_US)
        elif speed_mps < 0:
            # Reverse: -MAX_SPEED_MPS to 0 → 1300 to 1500 us
            pwm = self.map_range(speed_mps, -MAX_SPEED_MPS, 0.0, 
                                ESC_REVERSE_MAX_US, ESC_NEUTRAL_US)
        else:
            # Stop: neutral
            pwm = ESC_NEUTRAL_US
        
        return int(pwm)
    
    def convert_steering_to_pwm(self, angle_rad):
        """Convert steering angle in radians to PWM pulse width"""
        return int(self.map_range(angle_rad, 
                                  -MAX_STEERING_RAD, MAX_STEERING_RAD,
                                  SERVO_CENTER_US - SERVO_RANGE_US, 
                                  SERVO_CENTER_US + SERVO_RANGE_US))
    
    def drive_callback(self, msg):
        """Handle incoming drive commands"""
        if self.calibrating:
            self.get_logger().info("Ignoring commands during calibration...")
            return
        
        try:
            # Convert commands to PWM
            speed_pwm = self.convert_speed_to_pwm(msg.drive.speed)
            steering_pwm = self.convert_steering_to_pwm(msg.drive.steering_angle)
            
            # Convert to gpiozero value (0.0-1.0)
            speed_value = self._us_to_value(speed_pwm)
            steering_value = self._us_to_value(steering_pwm)
            
            # Send to hardware
            self.esc_pwm.value = speed_value
            self.servo_pwm.value = steering_value
            
            # Determine direction
            if msg.drive.speed > 0:
                direction = "FORWARD"
            elif msg.drive.speed < 0:
                direction = "REVERSE"
            else:
                direction = "STOP"
            
            self.get_logger().info(
                f"{direction} | Speed={msg.drive.speed:.2f}m/s→{speed_pwm}us | "
                f"Steering={msg.drive.steering_angle:.2f}rad→{steering_pwm}us"
            )
            
        except Exception as e:
            self.get_logger().error(f"Drive callback error: {e}")
    
    def on_shutdown(self):
        """Clean shutdown"""
        try:
            self.get_logger().info("Shutting down - setting neutral positions...")
            
            # Set neutral (STOP)
            self.esc_pwm.value = self._us_to_value(ESC_NEUTRAL_US)
            self.servo_pwm.value = self._us_to_value(SERVO_CENTER_US)
            
            time.sleep(0.5)
            
            # Off
            self.esc_pwm.off()
            self.servo_pwm.off()
            
            self.get_logger().info("GPIO shutdown complete")
            
        except Exception as e:
            self.get_logger().error(f"Shutdown error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RCCarPWMDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
