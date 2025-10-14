#!/usr/bin/env python3
#"useing this code to convert drive_msg to motor controls"
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

# --- Configuration Constants ---

# Steering Servo (Standard hobby servo control)
SERVO_CENTER_US = 1500   # Pulse width for straight ahead (microseconds)
SERVO_RANGE_US = 500     # Max deviation from center (e.g., 1000us to 2000us total range)
MAX_STEERING_RAD = 0.52  # Max physical steering angle in radians (matching the controller)

# Motor ESC (Electronic Speed Controller)
ESC_NEUTRAL_US = 1500    # Pulse width for STOP (microseconds)
ESC_MAX_FORWARD_US = 2000# Pulse width for full forward speed
ESC_MAX_REVERSE_US = 1000# Pulse width for full reverse speed
MAX_SPEED_MPS = 3.0      # Max speed in meters/second (matching the controller)

class RCCarPWMSimulator(Node):
    """
    Subscribes to AckermannDriveStamped messages and converts the speed 
    and steering_angle fields into simulated PWM pulse widths for the ESC and Servo.
    """
    def __init__(self):
        super().__init__('rc_car_pwm_simulator')
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'ackermann_cmd',
            self.drive_callback,
            10
        )
        self.get_logger().info('RC Car PWM Driver started. Subscribing to /ackermann_cmd.')
        self.current_speed_pwm = ESC_NEUTRAL_US
        self.current_steering_pwm = SERVO_CENTER_US

    def map_range(self, value, from_min, from_max, to_min, to_max):
        """Linearly maps a value from one range to another."""
        # Clamp the value to ensure it stays within the defined source range
        clamped_value = np.clip(value, from_min, from_max)
        
        # Perform the linear interpolation
        return (clamped_value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def convert_steering_to_pwm(self, angle_rad):
        """
        Converts a signed steering angle (rad) to a PWM pulse width (us).
        
        -MAX_STEERING_RAD maps to SERVO_CENTER_US - SERVO_RANGE_US (Full Left)
        +MAX_STEERING_RAD maps to SERVO_CENTER_US + SERVO_RANGE_US (Full Right)
        0.0 maps to SERVO_CENTER_US (Straight)
        """
        
        # 1. Scale the angle into the range [-1.0, 1.0]
        # This is essentially 'steering_percentage'
        steering_percentage = angle_rad / MAX_STEERING_RAD
        
        # 2. Map the percentage [-1.0, 1.0] to pulse width [1000us, 2000us]
        # We use map_range to convert [-1.0, 1.0] to [-SERVO_RANGE_US, +SERVO_RANGE_US] offset from center
        pulse_offset = steering_percentage * SERVO_RANGE_US
        
        # 3. Calculate final pulse width
        final_pulse = SERVO_CENTER_US + pulse_offset
        
        # Ensure the pulse width stays within physical limits
        return int(np.clip(final_pulse, SERVO_CENTER_US - SERVO_RANGE_US, SERVO_CENTER_US + SERVO_RANGE_US))

    def convert_speed_to_pwm(self, speed_mps):
        """
        Converts a signed speed (m/s) to an ESC PWM pulse width (us).
        
        +MAX_SPEED_MPS maps to ESC_MAX_FORWARD_US
        -MAX_SPEED_MPS maps to ESC_MAX_REVERSE_US
        0.0 maps to ESC_NEUTRAL_US (STOP)
        """
        
        # Clamp speed input to maximum limits
        speed_mps = np.clip(speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS)
        
        if speed_mps > 0:
            # Forward motion: Map [0, MAX_SPEED_MPS] to [ESC_NEUTRAL_US, ESC_MAX_FORWARD_US]
            return self.map_range(speed_mps, 
                                  0.0, MAX_SPEED_MPS, 
                                  ESC_NEUTRAL_US, ESC_MAX_FORWARD_US)
        elif speed_mps < 0:
            # Reverse motion: Map [-MAX_SPEED_MPS, 0] to [ESC_MAX_REVERSE_US, ESC_NEUTRAL_US]
            return self.map_range(speed_mps, 
                                  -MAX_SPEED_MPS, 0.0, 
                                  ESC_MAX_REVERSE_US, ESC_NEUTRAL_US)
        else:
            # Stopped
            return ESC_NEUTRAL_US

    def drive_callback(self, msg):
        """
        Processes incoming AckermannDriveStamped messages.
        """
        cmd_speed = msg.drive.speed
        cmd_steering = msg.drive.steering_angle
        
        # 1. Convert commands to PWM signals
        self.current_speed_pwm = self.convert_speed_to_pwm(cmd_speed)
        self.current_steering_pwm = self.convert_steering_to_pwm(cmd_steering)
        
        # 2. Simulate hardware output (printing to console)
        self.get_logger().info(
            f'Received: Speed={cmd_speed:.2f} m/s, Steering={cmd_steering:.2f} rad '
            f'| PWM Output: Speed={self.current_speed_pwm} us, Steering={self.current_steering_pwm} us'
        )

def main(args=None):
    rclpy.init(args=args)
    pwm_simulator = RCCarPWMSimulator()
    
    try:
        rclpy.spin(pwm_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        # Optional: Send a final stop command when shutting down
        pwm_simulator.get_logger().info('Shutting down. Sending neutral PWM signals.')
        # In a real driver, you would send SERVO_CENTER_US and ESC_NEUTRAL_US here.
        pwm_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
