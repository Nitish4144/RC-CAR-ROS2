# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64
# import Adafruit_PCA9685

# STEERING_CHANNEL = 0    # PCA9685 channel for steering servo
# ENGINE_CHANNEL   = 1    # PCA9685 channel for rear wheel servo ("engine")

# class RCCarDriver(Node):
#     def __init__(self):
#         super().__init__('rc_car_driver')
#         self.pwm = Adafruit_PCA9685.PCA9685()
#         self.pwm.set_pwm_freq(50)  # 50 Hz standard for servos

#         # Subscribe to steering and engine command topics
#         self.create_subscription(Float64, 'steering_cmd', self.steering_callback, 10)
#         self.create_subscription(Float64, 'throttle_cmd', self.engine_callback, 10)

#     def steering_callback(self, msg):
#         # Map input -1.0(left) ... 0.0(center) ... +1.0(right) to PWM
#         norm = max(-1.0, min(1.0, msg.data))
#         min_pwm = 300    # full left
#         mid_pwm = 400    # center
#         max_pwm = 500    # full right
#         pwm_value = int(mid_pwm + (max_pwm - mid_pwm) * norm) if norm > 0 else int(mid_pwm + (min_pwm - mid_pwm) * norm)
#         self.pwm.set_pwm(STEERING_CHANNEL, 0, pwm_value)

#     def engine_callback(self, msg):
#         # Map -1.0(reverse) ... 0.0(stop) ... +1.0(forward) to PWM for rear wheel servo
#         norm = max(-1.0, min(1.0, msg.data))
#         min_pwm = 300    # full reverse
#         mid_pwm = 400    # stop/neutral
#         max_pwm = 500    # full forward
#         pwm_value = int(mid_pwm + (max_pwm - mid_pwm) * norm) if norm > 0 else int(mid_pwm + (min_pwm - mid_pwm) * norm)
#         self.pwm.set_pwm(ENGINE_CHANNEL, 0, pwm_value)

# def main(args=None):
#     rclpy.init(args=args)
#     node = RCCarDriver()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()



#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped # Import the specific message

class SimpleCarController(Node):
    def __init__(self):
        super().__init__('ackermann_car_controller')
        
        # New: Publisher for the combined Ackermann drive command
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10) 
        
        # Original: Joystick subscriber
        self.joystick_sub = self.create_subscription(Joy, 'driftpilot_joy/joy_ramped', self.joy_callback, 10)
        
        self.current_speed = 0.0      # Corresponds to speed (m/s)
        self.current_steering = 0.0   # Corresponds to steering_angle (rad)
        
        # Define maximum movement values for scaling
        self.MAX_SPEED = 3.0           # Max linear speed in meters/second
        self.MAX_STEERING_ANGLE = 0.52 # Max steering angle in radians (approx 30 degrees)
        
        self.timer = self.create_timer(1.0 / 60.0, self.publish_commands) # 60Hz update

    def joy_callback(self, msg):
        # Safety check: Ensure the message has enough axes to read
        # Checking for index 4 requires a length of at least 5.
        if len(msg.axes) >= 5: 
            
            # --- Input Mapping ---
            # Axis 0: Left/Right Stick X-axis (for Steering)
            # Axis 4: Trigger/Stick (for Throttle/Speed) - Used based on your original code
            
            raw_steering_input = msg.axes[0]
            raw_throttle_input = msg.axes[4] 
            
            # 1. Calculate Speed (Linear): Maps [-1.0, 1.0] input to [-MAX_SPEED, MAX_SPEED]
            # Assumes stick up/down maps to forward/backward speed
            self.current_speed = ((1+raw_throttle_input)/2) * self.MAX_SPEED

            # 2. Calculate Steering Angle (Angular): Maps [-1.0, 1.0] input to [-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE]
            steering_magnitude = abs(raw_steering_input)

            # . Scale the magnitude to the MAX_STEERING_ANGLE
            # The angle is always positive, representing how far the wheels are turned.
            # Example: -0.5 input -> 0.5 magnitude -> 0.26 rad
            # Example: +0.5 input -> 0.5 magnitude -> 0.26 rad
            angle_magnitude = steering_magnitude * self.MAX_STEERING_ANGLE
            
            # . Apply the sign back for the AckermannDriveStamped message:
            # This ensures the Ackermann message is properly signed, which is standard.
            # If you truly need unipolar output, you would publish a separate direction signal
            # and set self.current_steering = angle_magnitude
            
            if raw_steering_input < 0:
                self.current_steering = -angle_magnitude # Turn Left
            else:
                self.current_steering = angle_magnitude  # Turn Right or Straight (if input is 0)
        else:
            # Emergency stop/neutral if the joystick data is invalid
            self.current_speed = 0.0
            self.current_steering = 0.0

    def publish_commands(self):
        # 1. Create a new AckermannDriveStamped message
        drive_msg = AckermannDriveStamped()
        
        # 2. Fill the Header
        # The 'Stamped' version requires a header for timestamping and frame_id
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link' # Replace with your robot's base frame
        
        # 3. Fill the Drive command fields
        drive_msg.drive.speed = self.current_speed             # Set linear speed (m/s)
        drive_msg.drive.steering_angle = self.current_steering # Set steering angle (rad)
        
        # 4. Publish the command
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()