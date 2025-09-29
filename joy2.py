#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class SimpleCarController(Node):
    def __init__(self):
        super().__init__('simple_car_controller')
        self.steering_pub = self.create_publisher(Float64, 'steering_cmd', 10)
        self.throttle_pub = self.create_publisher(Float64, 'throttle_cmd', 10)
        self.joystick_sub = self.create_subscription(Joy, 'catatron_joy/joy_ramped', self.joy_callback, 10)
        self.steering = 0.0  # Default: center
        self.throttle = 0.0  # Default: neutral
        self.timer = self.create_timer(1.0 / 60.0, self.publish_commands)  # 60Hz update

    def joy_callback(self, msg):
        # Example: axis 0 = steering (left/right), axis 1 = throttle (up/down)
        if len(msg.axes) >= 2:
            self.steering = msg.axes     # Left/right stick for steering [-1.0, 1.0]
            self.throttle = msg.axes[4]     # Up/down stick for throttle [-1.0, 1.0]
        else:
            self.steering = 0.0
            self.throttle = 0.0

    def publish_commands(self):
        self.steering_pub.publish(Float64(data=self.steering))
        self.throttle_pub.publish(Float64(data=self.throttle))

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
