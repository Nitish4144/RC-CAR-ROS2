#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class PS4Controller(Node):
    def __init__(self, rate):
        super().__init__('joystick_ramped')
        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Joy, 'driftpilot_joy/joy_ramped', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_joy)
        self.target_joy = Joy()
        self.target_joy.axes = [0., 0., 1., 0., 0., 1., 0., 0.]
        self.target_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def joy_callback(self, msg):
        self.target_joy.buttons = msg.buttons
        self.target_joy.axes = msg.axes

    def publish_joy(self):
        self.publisher.publish(self.target_joy)

def main(args=None):
    rclpy.init(args=args)
    joystick = PS4Controller(rate=30)
    rclpy.spin(joystick)
    joystick.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
