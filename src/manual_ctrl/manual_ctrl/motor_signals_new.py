#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import serial
import time
import math

# ================= SERIAL CONFIG =================
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

# ================= VEHICLE PARAMS =================
MAX_SPEED_MPS = 1.0
MAX_REVERSE_SPEED_MPS = -1.0
MAX_STEERING_RAD = 0.8

ESC_NEUTRAL_US = 1500
ESC_MIN_US = 1320
ESC_FORWARD_MAX_US = 1680

SERVO_CENTER_US = 1500
SERVO_RANGE_US = 200
# =================================================


class MotorSignalsNode(Node):
    def __init__(self):
        super().__init__('motor_signals_node')

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            time.sleep(2)
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

        # 🔥 NEW: Track previous speed sign
        self.previous_speed = 0.0
        self.reverse_armed = False

    def map_range(self, val, in_min, in_max, out_min, out_max):
        mapped = out_min + (val - in_min) * (out_max - out_min) / (in_max - in_min)
        return max(min(mapped, out_max), out_min)

    def drive_callback(self, msg):

        target_speed = msg.drive.speed
        target_angle = msg.drive.steering_angle

        # ================= ESC LOGIC FIX =================

        # Detect Forward → Reverse transition
        if self.previous_speed > 0 and target_speed < 0 and not self.reverse_armed:
            self.get_logger().info("Forward → Reverse detected. Sending neutral reset...")
            self.ser.write(f"{ESC_NEUTRAL_US},{SERVO_CENTER_US}\n".encode())
            time.sleep(0.25)   # 250ms delay for 1/10 RC ESC
            self.reverse_armed = True

        # Reset reverse arm when returning to neutral or forward
        if target_speed >= 0:
            self.reverse_armed = False

        # =================================================

        esc_us = int(self.map_range(
            target_speed,
            MAX_REVERSE_SPEED_MPS, MAX_SPEED_MPS,
            ESC_MIN_US, ESC_FORWARD_MAX_US
        ))

        steer_us = int(self.map_range(
            target_angle,
            -MAX_STEERING_RAD, MAX_STEERING_RAD,
            SERVO_CENTER_US - SERVO_RANGE_US,
            SERVO_CENTER_US + SERVO_RANGE_US
        ))

        self.get_logger().info(
            f"ACKERMANN: {target_speed:.2f} m/s, {target_angle:.2f} rad -> ESC: {esc_us}, SERVO: {steer_us}"
        )

        cmd = f"{esc_us},{steer_us}\n"
        self.ser.write(cmd.encode())

        self.previous_speed = target_speed

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.write(f"{ESC_NEUTRAL_US},{SERVO_CENTER_US}\n".encode())
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
