#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import serial
import time

# ================= SERIAL CONFIG =================
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

# ================= VEHICLE PARAMS =================
# Steering Params
SERVO_CENTER_US = 1500
SERVO_RANGE_US = 200
MAX_STEERING_RAD = 0.27  # Approx 15.5 degrees

# Bidirectional ESC Params
ESC_CENTER_US = 1500     # Neutral
ESC_MAX_FORWARD = 1900   # Full Forward
ESC_MAX_REVERSE = 1100   # Full Reverse
MAX_SPEED_MPS = 1.0      # Target speed at max PWM

# Staged mapping offsets (relative to center)
# These define the "steps" in power for smoother low-speed control
STAGE_1_OFFSET = 100   # 1600 Fwd / 1400 Rev
STAGE_2_OFFSET = 250   # 1750 Fwd / 1250 Rev
STAGE_3_OFFSET = 350   # 1850 Fwd / 1150 Rev

class AckermannSerialDriver(Node):
    def __init__(self):
        super().__init__('ackermann_serial_driver')

        # Initialize Serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            time.sleep(2)  # Wait for Arduino bootloader
            self.get_logger().info(f"✓ Connected to Arduino on {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().fatal(f"Serial connection failed: {e}")
            raise e

        # Subscriber: Listening for Ackermann commands
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'ackermann_cmd',
            self.drive_callback,
            10
        )

    def map_range(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def convert_speed_to_pwm(self, speed_mps):
        """
        Maps speed to PWM for a bidirectional ESC.
        Positive = Forward (1500 to 1900)
        Negative = Reverse (1500 to 1100)
        """
        if abs(speed_mps) < 0.01:
            return ESC_CENTER_US

        # Determine direction and work with absolute speed for mapping
        is_forward = speed_mps > 0
        abs_speed = abs(speed_mps)
        ratio = abs_speed / MAX_SPEED_MPS

        # Select stage limits based on direction
        if is_forward:
            s0, s1, s2, s3, s_max = (ESC_CENTER_US, 
                                     ESC_CENTER_US + STAGE_1_OFFSET, 
                                     ESC_CENTER_US + STAGE_2_OFFSET, 
                                     ESC_CENTER_US + STAGE_3_OFFSET, 
                                     ESC_MAX_FORWARD)
        else:
            s0, s1, s2, s3, s_max = (ESC_CENTER_US, 
                                     ESC_CENTER_US - STAGE_1_OFFSET, 
                                     ESC_CENTER_US - STAGE_2_OFFSET, 
                                     ESC_CENTER_US - STAGE_3_OFFSET, 
                                     ESC_MAX_REVERSE)

        # Staged Logic
        if ratio <= 0.1:
            return int(self.map_range(abs_speed, 0.0, MAX_SPEED_MPS, s0, s1))
        elif ratio <= 0.45:
            return int(self.map_range(abs_speed, 0.0, MAX_SPEED_MPS, s1, s2))
        elif ratio <= 0.75:
            return int(self.map_range(abs_speed, 0.0, MAX_SPEED_MPS, s2, s3))
        else:
            return int(self.map_range(abs_speed, 0.0, MAX_SPEED_MPS, s3, s_max))

    def convert_steering_to_pwm(self, angle_rad):
        # Linear steering mapping
        return int(self.map_range(
            angle_rad,
            -MAX_STEERING_RAD, MAX_STEERING_RAD,
            SERVO_CENTER_US - SERVO_RANGE_US,
            SERVO_CENTER_US + SERVO_RANGE_US
        ))

    def drive_callback(self, msg):
        speed_pwm = self.convert_speed_to_pwm(msg.drive.speed)
        steer_pwm = self.convert_steering_to_pwm(msg.drive.steering_angle)

        # Build command: "SPEED_PWM,STEER_PWM\n"
        cmd = f"{speed_pwm},{steer_pwm}\n"
        
        try:
            self.ser.write(cmd.encode())
            # self.get_logger().info(f"Sent: {cmd.strip()}") 
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def on_shutdown(self):
        # Safety: Set to neutral/center on exit
        if self.ser.is_open:
            stop_cmd = f"{ESC_CENTER_US},{SERVO_CENTER_US}\n"
            self.ser.write(stop_cmd.encode())
            time.sleep(0.1)
            self.ser.close()
        self.get_logger().info("Serial connection closed safely.")

def main(args=None):
    rclpy.init(args=args)
    node = AckermannSerialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
