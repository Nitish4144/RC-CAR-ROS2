


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped # Import the specific message
GEAR_UP_BTN   = 4
GEAR_DOWN_BTN = 5
BRAKE_BTN     = 1




class SimpleCarController(Node):
    def __init__(self):
        super().__init__('ackermann_car_controller')
        
        # New: Publisher for the combined Ackermann drive command
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10) 
        
        # Original: Joystick subscriber
        self.joystick_sub = self.create_subscription(Joy, 'driftpilot_joy/joy_ramped', self.joy_callback, 10)
        
        self.current_speed = 0.0      # Corresponds to speed (m/s)
        self.current_steering = 0.0   # Corresponds to steering_angle (rad)
        # ---------- Gear state ----------
        # ---------- Gear state ----------
        self.gear = 1              # START FROM GEAR 1
        self.MAX_GEAR = 3
        self.THROTTLE_AXIS = 1
        self.STEERING_AXIS = 2


        self.gear_changing = False
        self.gear_timer = 0.0
        self.GEAR_LOCK_TIME = 0.5  # seconds

        # ---------- Throttle zones ----------
        self.THROTTLE_LOW  = 0.2
        self.THROTTLE_HIGH = 0.8

        # ---------- Gear speed limits ----------
        self.GEAR_MAX_SPEED = {
            1: 0.3,
            2: 0.6,
            3: 1.0
        }


        # Define maximum movement values for scaling
        self.MAX_SPEED = 1             # Max linear speed in meters/second
        self.MAX_STEERING_ANGLE = 0.27  # Max steering angle in radians (approx 30 degrees)
        
        self.timer = self.create_timer(1.0 / 50.0, self.publish_commands) # 50Hz update

    def joy_callback(self, msg):
        # Safety check: Ensure the message has enough axes to read
        # Checking for index 4 requires a length of at least 5.
        throttle = msg.axes[self.THROTTLE_AXIS]
        steering = msg.axes[self.STEERING_AXIS]

        gear_up   = msg.buttons[GEAR_UP_BTN]
        gear_down = msg.buttons[GEAR_DOWN_BTN]
        brake     = msg.buttons[BRAKE_BTN]
        
        if brake:
            self.current_speed = 0.0
            return
            
        if gear_up and throttle > self.THROTTLE_HIGH and not self.gear_changing:
            if self.gear < self.MAX_GEAR:
                self.gear += 1
                self.gear_changing = True
                self.gear_timer = self.GEAR_LOCK_TIME
                
        if gear_down and throttle < self.THROTTLE_LOW and not self.gear_changing:
            if self.gear > 1:
                self.gear -= 1
                self.gear_changing = True
                self.gear_timer = self.GEAR_LOCK_TIME
                
        if self.gear_changing:
            self.current_speed = 0.0
            return




           
       
            # 1. Calculate Speed (Linear): Maps [-1.0, 1.0] input to [-MAX_SPEED, MAX_SPEED]
            # Assumes stick up/down maps to forward/backward speed
            max_speed = self.GEAR_MAX_SPEED[self.gear]
            self.current_speed = throttle * max_speed


            # 2. Calculate Steering Angle (Angular): Maps [-1.0, 1.0] input to [-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE]
            self.current_steering = steering * self.MAX_STEERING_ANGLE
            
        else:# Emergency stop/neutral if the joystick data is invalid
            self.current_speed = 0.0
            self.current_steering = 0.0
        max_speed = self.GEAR_MAX_SPEED[self.gear]
        self.current_speed = throttle * max_speed
        self.current_steering = steering * self.MAX_STEERING_ANGLE

    def publish_commands(self):
       
        # 1. Create a new AckermannDriveStamped message
        dt = 1.0 / 50.0
        if self.gear_changing:
            self.gear_timer -= dt
            if self.gear_timer <= 0:
                self.gear_changing = False

        drive_msg = AckermannDriveStamped()
        
        # 2. Fill the Header
        # The 'Stamped' version requires a header for timestamping and frame_id
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link' # Replace with your robot's base frame
        
        # 3. Fill the Drive command fields
        drive_msg.drive.speed = self.current_speed             # Set linear speed (m/s)
        drive_msg.drive.steering_angle = self.current_steering # Set steering angle (rad)
        drive_msg.drive.acceleration = float(self.gear)

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
