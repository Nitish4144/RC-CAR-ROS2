#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import serial
import time

# ================= SERIAL CONFIG =================
SERIAL_PORT = "/dev/ttyACM0"   # change if needed
BAUDRATE = 115200

# ================= VEHICLE PARAMS =================
MAX_SPEED_MPS = 1.0
MAX_STEERING_RAD = 0.8
ESC_NEUTRAL_US = 1000
ESC_FORWARD_MAX_US = 1600

SERVO_CENTER_US = 1500
SERVO_RANGE_US = 200

# =================================================


class MotorSignalsNode(Node):
    def __init__(self):
        super().__init__('motor_signals_node')

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            time.sleep(2)  # Arduino reset delay
            self.get_logger().info("✓ Connected to Arduino over serial")
        except Exception as e:
            self.get_logger().fatal(f"Serial connection failed: {e}")
            raise e

        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'ackermann_cmd',
            self.drive_callback,
            10
        )

    def map_range(self, val, in_min, in_max, out_min, out_max):
        return out_min + (val - in_min) * (out_max - out_min) / (in_max - in_min)

    def drive_callback(self, msg):
        speed = max(0.0, min(msg.drive.speed, MAX_SPEED_MPS))
        steer = max(-MAX_STEERING_RAD,
                    min(msg.drive.steering_angle, MAX_STEERING_RAD))
        self.get_logger().info(f"speed: {speed}")
        self.get_logger().info(f"steer: {steer}")

     

        esc_us = int(self.map_range(
            speed,
            0.0, MAX_SPEED_MPS,
            ESC_NEUTRAL_US, ESC_FORWARD_MAX_US
        ))

        steer_us = int(self.map_range(
            steer,
            -MAX_STEERING_RAD, MAX_STEERING_RAD,
            SERVO_CENTER_US - SERVO_RANGE_US,
            SERVO_CENTER_US + SERVO_RANGE_US
        ))

        cmd = f"{esc_us},{steer_us}\n"
        self.ser.write(cmd.encode())

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorSignalsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
