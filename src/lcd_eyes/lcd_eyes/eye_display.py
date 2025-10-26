#!/usr/bin/env python3
import time

# --- Try ROS imports (will fail gracefully if ROS2 not installed) ---
try:
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    USE_ROS = True
except ImportError:
    USE_ROS = False

# --- Graphics ---
try:
    import board, busio
    import adafruit_ssd1306
    from PIL import Image, ImageDraw
    USE_OLED = True
except Exception:
    from PIL import Image, ImageDraw, ImageTk
    import tkinter as tk
    USE_OLED = False


class EyeDisplayBase:
    def __init__(self):
        self.target_angle = 0.0
        self.current_angle = 0.0
        self.target_speed = 0.0
        self.current_speed = 0.0
        self.last_update = time.time()

        if USE_OLED:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.disp_left = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3C)
            self.disp_right = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3D)
            for d in [self.disp_left, self.disp_right]:
                d.fill(0)
                d.show()
        else:
            self.root = tk.Tk()
            self.root.title("LCD Eyes Simulation")
            self.canvas = tk.Canvas(self.root, width=300, height=150, bg="black")
            self.canvas.pack()
            self.root.update()

    def update_display(self):
        dt = time.time() - self.last_update
        self.last_update = time.time()
        alpha = 0.2
        self.current_angle += alpha * (self.target_angle - self.current_angle)
        self.current_speed += alpha * (self.target_speed - self.current_speed)

        vanish_side = None
        if self.current_angle > 0.17:
            vanish_side = 'right'
        elif self.current_angle < -0.17:
            vanish_side = 'left'
        if abs(self.current_speed) < 0.1 and abs(self.current_angle) < 0.1:
            vanish_side = 'top'

        base_r, min_r = 25, 15
        s = min(abs(self.current_speed) / 3.0, 1.0)
        radius = int(base_r - s * (base_r - min_r))

        if USE_OLED:
            self.draw_oled_eye(self.disp_left, vanish_side, radius)
            self.draw_oled_eye(self.disp_right, vanish_side, radius)
        else:
            self.draw_sim(radius, vanish_side)

    def draw_oled_eye(self, disp, vanish_side, radius):
        img = Image.new('1', (disp.width, disp.height))
        draw = ImageDraw.Draw(img)
        cx, cy = disp.width // 2, disp.height // 2
        bbox = [cx - radius, cy - radius, cx + radius, cy + radius]
        draw.ellipse(bbox, outline=255, fill=255)
        if vanish_side == 'right':
            draw.rectangle([cx, 0, disp.width, disp.height], fill=0)
        elif vanish_side == 'left':
            draw.rectangle([0, 0, cx, disp.height], fill=0)
        elif vanish_side == 'top':
            draw.rectangle([0, 0, disp.width, cy], fill=0)
        disp.image(img)
        disp.show()

    def draw_sim(self, radius, vanish_side):
        self.canvas.delete("all")
        cx1, cy1, cx2, cy2 = 90, 75, 210, 75
        for cx in [cx1, cx2]:
            x0, y0, x1, y1 = cx - radius, cy1 - radius, cx + radius, cy1 + radius
            self.canvas.create_oval(x0, y0, x1, y1, fill="white")
            if vanish_side == 'right':
                self.canvas.create_rectangle(cx, 0, x1, 150, fill="black", outline="")
            elif vanish_side == 'left':
                self.canvas.create_rectangle(0, 0, cx, 150, fill="black", outline="")
            elif vanish_side == 'top':
                self.canvas.create_rectangle(0, 0, 300, cy1, fill="black", outline="")
        self.root.update()


# --- ROS Node version (only active if ROS2 is available) ---
if USE_ROS:
    class EyeDisplayNode(Node, EyeDisplayBase):
        def __init__(self):
            Node.__init__(self, 'eye_display')
            EyeDisplayBase.__init__(self)
            self.create_subscription(AckermannDriveStamped, '/drive', self.drive_callback, 10)
            self.create_timer(0.05, self.update_display)
            self.get_logger().info("LCD Eyes Node started 👀")

        def drive_callback(self, msg):
            self.target_angle = msg.drive.steering_angle
            self.target_speed = msg.drive.speed


def main():
    if USE_ROS:
        rclpy.init()
        node = EyeDisplayNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()
    else:
        # Standalone simulation mode
        eyes = EyeDisplayBase()
        print("Running in standalone mode (no ROS2 detected).")
        try:
            while True:
                eyes.update_display()
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

