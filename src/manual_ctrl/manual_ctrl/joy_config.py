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

        self.joystick_sub = self.create_subscription(
            Joy,
            'driftpilot_joy/joy_ramped',
            self.joy_callback,
            10
        )

        # ---------- State ----------
        self.steering = 0.0                # direct steering
        self.throttle_target = 0.0         # joystick command
        self.throttle_output = 0.0         # ramped output

        # ---------- RAMP SETTINGS ----------
        self.THROTTLE_RAMP_RATE = 0.03     # max change per cycle (60 Hz)

        self.timer = self.create_timer(1.0 / 60.0, self.publish_commands)

    def joy_callback(self, msg):
        if len(msg.axes) >= 5:
            self.throttle_target = msg.axes[4]  # throttle stick
            self.steering = msg.axes[0]         # steering stick
        else:
            self.throttle_target = 0.0
            self.steering = 0.0

    def ramp(self, target, current, rate):
        delta = target - current
        if delta > rate:
            delta = rate
        elif delta < -rate:
            delta = -rate
        return current + delta

    def publish_commands(self):
        # ---- Apply ramp ONLY to throttle ----
        self.throttle_output = self.ramp(
            self.throttle_target,
            self.throttle_output,
            self.THROTTLE_RAMP_RATE
        )

        self.steering_pub.publish(Float64(data=self.steering))
        self.throttle_pub.publish(Float64(data=self.throttle_output))


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
