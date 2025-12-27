#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from gpiozero import PWMOutputDevice
import time
import threading

# GPIO pins (BCM numbering)
ESC_GPIO_PIN = 18 
SERVO_GPIO_PIN = 17

# Servo parameters
SERVO_CENTER_US = 1500
SERVO_RANGE_US = 500
MAX_STEERING_RAD = 0.52

# ===== ESC PARAMETERS (UNI-DIRECTIONAL) =====
ESC_NEUTRAL_US = 1000        # STOP
ESC_FORWARD_MAX_US = 1700   # MAX FORWARD
MAX_SPEED_MPS = 0.5         # Reduced speed

PWM_FREQUENCY = 50  # 50Hz

class RCCarPWMDriver(Node):
    def __init__(self):
        super().__init__('rc_car_pwm_driver')

        try:
            # PWM devices
            self.esc_pwm = PWMOutputDevice(ESC_GPIO_PIN, frequency=PWM_FREQUENCY)
            self.servo_pwm = PWMOutputDevice(SERVO_GPIO_PIN, frequency=PWM_FREQUENCY)
            # Safe startup
            self.esc_pwm.value = self._us_to_value(ESC_NEUTRAL_US)
            self.servo_pwm.value = self._us_to_value(SERVO_CENTER_US)

            self.get_logger().info(
                f"ESC Uni-directional | Neutral={ESC_NEUTRAL_US}us "
                f"Max={ESC_FORWARD_MAX_US}us"
            )

            # ESC calibration
            self.calibrating = True

            # ROS subscription
            self.state = 0
            self.get_logger().info("Starting ESC calibration...")
            threading.Thread(target=self._calibrate_esc, daemon=True).start()
            
            # Subscribe to drive commands
            self.subscription = self.create_subscription(
                AckermannDriveStamped,
                'ackermann_cmd',
                self.drive_callback,
                10
            )

        except Exception as e:
            self.get_logger().error(f"Initialization failed: {e}")
            raise

    # ===== ESC CALIBRATION =====
    def _calibrate_esc(self):
        try:
            self.get_logger().info("ESC Calibration: Max Forward")
            self.esc_pwm.value = self._us_to_value(ESC_FORWARD_MAX_US)
            time.sleep(5)

            self.get_logger().info("ESC Calibration: Neutral (1000us)")
            self.esc_pwm.value = self._us_to_value(ESC_NEUTRAL_US)
            time.sleep(1)
            self.calibrating = False
            self.get_logger().info("✓ ESC calibration complete")

        except Exception as e:
            self.get_logger().error(f"ESC calibration error: {e}")
            self.calibrating = False

    def _us_to_value(self, pulse_us):
        """Convert microseconds to gpiozero PWM value"""
        return pulse_us / 20000.0

    def map_range(self, value, in_min, in_max, out_min, out_max):
        value = np.clip(value, in_min, in_max)
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # ===== SPEED CONVERSION (FORWARD ONLY) =====
    def convert_speed_to_pwm(self, speed_mps):
        speed_mps = np.clip(speed_mps, 0.0, MAX_SPEED_MPS)

        pwm = self.map_range(
            speed_mps,
            0.0, MAX_SPEED_MPS,
            ESC_NEUTRAL_US, ESC_FORWARD_MAX_US
        )

        return int(pwm)

    def convert_steering_to_pwm(self, angle_rad):
        return int(self.map_range(
            angle_rad,
            -MAX_STEERING_RAD, MAX_STEERING_RAD,
            SERVO_CENTER_US - SERVO_RANGE_US,
            SERVO_CENTER_US + SERVO_RANGE_US
        ))

    def drive_callback(self, msg):
        if self.calibrating:
            self.get_logger().info(f"Ignoring commands during calibration...{self.state}")
            return

        try:
            speed_pwm = self.convert_speed_to_pwm(msg.drive.speed)
            steering_pwm = self.convert_steering_to_pwm(msg.drive.steering_angle)

            self.esc_pwm.value = self._us_to_value(speed_pwm)
            self.servo_pwm.value = self._us_to_value(steering_pwm)

            self.get_logger().info(
                f"FORWARD | Speed={msg.drive.speed:.2f} → {speed_pwm}us | "
                f"Steering={steering_pwm}us"
            )

        except Exception as e:
            self.get_logger().error(f"Drive callback error: {e}")

    def on_shutdown(self):
        try:
            self.get_logger().info("Shutdown: neutral")
            self.esc_pwm.value = self._us_to_value(ESC_NEUTRAL_US)
            self.servo_pwm.value = self._us_to_value(SERVO_CENTER_US)
            time.sleep(0.5)
            self.esc_pwm.off()
            self.servo_pwm.off()
        except Exception as e:
            self.get_logger().error(f"Shutdown error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RCCarPWMDriver()

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
