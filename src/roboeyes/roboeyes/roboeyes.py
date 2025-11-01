#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
import time
import board
import busio
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw

class OLEDEyes(Node):
    def __init__(self):
        super().__init__('oled_eyes_node')

        # --- OLED Setup ---
        i2c = busio.I2C(board.SCL, board.SDA)
        self.display = SSD1306_I2C(128, 64, i2c)
        self.display.fill(0)
        self.display.show()

        # --- Eye parameters ---
        self.center_x = 64
        self.center_y = 32
        self.eye_w = 40
        self.eye_h = 25
        self.min_w = 15
        self.min_h = 10
        self.border = 6

        self.target_x = self.center_x
        self.last_cmd_time = time.time()
        self.random_mode = False
        self.steering = 0.0
        self.throttle = 0.0

        # ROS2 subscription
        self.create_subscription(AckermannDrive, '/ackermann_cmd', self.callback, 10)

        # Timer for updates
        self.timer = self.create_timer(0.05, self.update_display)

        self.get_logger().info("OLED Eyes Node Started ✅")

    def callback(self, msg: AckermannDrive):
        self.steering = msg.steering_angle  # typically between -0.4 and 0.4
        self.throttle = msg.speed
        self.last_cmd_time = time.time()
        self.random_mode = False

    def draw_eye(self, x, y, w, h):
        img = Image.new('1', (128, 64))
        draw = ImageDraw.Draw(img)
        x0 = int(x - w / 2)
        y0 = int(y - h / 2)
        x1 = int(x + w / 2)
        y1 = int(y + h / 2)
        draw.rounded_rectangle([x0, y0, x1, y1], radius=self.border, fill=255)
        self.display.image(img)
        self.display.show()

    def update_display(self):
        now = time.time()

        if now - self.last_cmd_time > 10:
            # Random motion every 2–3 sec if idle
            if not self.random_mode:
                self.get_logger().info("Entering random motion mode")
            self.random_mode = True

        if self.random_mode:
            # Move randomly left/right
            offset = (time.time() * 100) % 50 - 25
            target_x = self.center_x + offset
        else:
            # Move based on steering
            target_x = self.center_x + (self.steering * 60)

        # Smooth transition
        self.center_x += (target_x - self.center_x) * 0.08

        # Shrink based on throttle
        scale = max(0.6, 1.0 - abs(self.throttle) * 0.5)
        self.eye_w = 40 * scale
        self.eye_h = 25 * scale

        self.draw_eye(self.center_x, self.center_y, self.eye_w, self.eye_h)

def main(args=None):
    rclpy.init(args=args)
    node = OLEDEyes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
