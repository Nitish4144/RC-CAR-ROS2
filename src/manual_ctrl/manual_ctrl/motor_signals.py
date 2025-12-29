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
ESC_STAGE1_US = 1250         # End of first half travel
ESC_FORWARD_MAX_US = 1600    # Max output for second half travel
MAX_SPEED_MPS = 0.5          # Command range scaling

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
                f"Stage1={ESC_STAGE1_US}us Max={ESC_FORWARD_MAX_US}us"
            )

            # ESC calibration
            self.calibrating = True
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
            
            self.get_logger().info("ESC Calibration: Neutral (1000us)")
            self.esc_pwm.value = self._us_to_value(ESC_NEUTRAL_US)
            time.sleep(2)

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

    # ===== TWO-STAGE SPEED CONVERSION (FORWARD ONLY) =====
    def convert_speed_to_pwm(self, speed_mps):
        # Clamp input to valid range
        speed_mps = np.clip(speed_mps, 0.0, MAX_SPEED_MPS)

        # Normalize to 0…1 joystick-equivalent travel
        t = speed_mps / MAX_SPEED_MPS

        if t <= 0.5:
            # First half → 1000 → 1250 (gentle ramp)
            pwm = self.map_range(
                t, 0.0, 0.5,
                ESC_NEUTRAL_US, ESC_STAGE1_US
            )
        else:
            # Second half → 1250 → 1600 (slower ramp)
            pwm = self.map_range(
                t, 0.5, 1.0,
                ESC_STAGE1_US, ESC_FORWARD_MAX_US
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
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import time
import threading
import smbus

# ================= PCA9685 CONFIG =================
I2C_ADDR = 0x40
ESC_CH = 0
SERVO_CH = 1

MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

# ================= SERVO / ESC PARAMS =================
SERVO_CENTER_US = 1500
SERVO_RANGE_US = 500
MAX_STEERING_RAD = 0.27 # 15.5 deg

ESC_NEUTRAL_US = 1000
ESC_FORWARD_MAX_US = 1700
ESC_CALIB_MAX_US = 2000
MAX_SPEED_MPS = 0.5

PWM_FREQ = 50  # Hz

# =====================================================

class RCCarPWMDriver(Node):
    def __init__(self):
        super().__init__('rc_car_pwm_driver')

        self.bus = smbus.SMBus(1)

        # 🔧 FIX: init PCA9685 properly
        self._init_pca9685()
        self.set_pwm_freq(PWM_FREQ)

        # Safe startup
        self.set_pwm_us(ESC_CH, ESC_NEUTRAL_US)
        self.set_pwm_us(SERVO_CH, SERVO_CENTER_US)

        self.calibrating = True
        self.state = 0

        self.get_logger().info("Starting ESC calibration...")
        threading.Thread(target=self._calibrate_esc, daemon=True).start()

        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'ackermann_cmd',
            self.drive_callback,
            10
        )

    # ================= PCA9685 INIT =================
    def _init_pca9685(self):
        # Reset
        self.bus.write_byte_data(I2C_ADDR, MODE1, 0x00)
        time.sleep(0.01)

        # Auto-increment + wake
        self.bus.write_byte_data(I2C_ADDR, MODE1, 0x20)
        time.sleep(0.01)

        self.get_logger().info("PCA9685 initialized")

    # ================= PCA9685 LOW LEVEL =================
    def write(self, reg, val):
        self.bus.write_byte_data(I2C_ADDR, reg, val)

    def set_pwm_freq(self, freq):
        prescale = int(25000000 / (4096 * freq) - 1)
        oldmode = self.bus.read_byte_data(I2C_ADDR, MODE1)

        self.write(MODE1, (oldmode & 0x7F) | 0x10)  # sleep
        self.write(PRESCALE, prescale)
        self.write(MODE1, oldmode)
        time.sleep(0.005)
        self.write(MODE1, oldmode | 0x80)

        self.get_logger().info(f"PCA9685 PWM freq set to {freq} Hz")

    def set_pwm_us(self, channel, us):
        ticks = int(us * 4096 / 20000)
        base = LED0_ON_L + 4 * channel

        self.write(base, 0)                 # ON_L
        self.write(base + 1, 0)             # ON_H
        self.write(base + 2, ticks & 0xFF)  # OFF_L
        self.write(base + 3, ticks >> 8)    # OFF_H

    # ================= ESC CALIBRATION =================
    def _calibrate_esc(self):
        try:
            

            self.state = 1
            self.get_logger().info("ESC CAL: NEUTRAL")
            self.set_pwm_us(ESC_CH, ESC_NEUTRAL_US)
            time.sleep(2)

            self.calibrating = False
            self.get_logger().info("✓ ESC calibration complete")

        except Exception as e:
            self.get_logger().error(f"ESC calibration error: {e}")
            self.calibrating = False

    # ================= MAPPING =================
    def map_range(self, value, in_min, in_max, out_min, out_max):
        value = np.clip(value, in_min, in_max)
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def convert_speed_to_pwm(self, speed_mps):
        speed_mps = np.clip(speed_mps, 0.0, MAX_SPEED_MPS)
        return int(self.map_range(
            speed_mps,
            0.0, MAX_SPEED_MPS,
            ESC_NEUTRAL_US, ESC_FORWARD_MAX_US
        ))

    def convert_steering_to_pwm(self, angle_rad):
        return int(self.map_range(
            angle_rad,
            -MAX_STEERING_RAD, MAX_STEERING_RAD,
            SERVO_CENTER_US - SERVO_RANGE_US,
            SERVO_CENTER_US + SERVO_RANGE_US
        ))

    # ================= ROS CALLBACK =================
    def drive_callback(self, msg):
        if self.calibrating:
            return

        speed_pwm = self.convert_speed_to_pwm(msg.drive.speed)
        steer_pwm = self.convert_steering_to_pwm(msg.drive.steering_angle)

        self.set_pwm_us(ESC_CH, speed_pwm)
        self.get_logger().info(f"speed-pwm:{speed_pwm}...")
        self.set_pwm_us(SERVO_CH, steer_pwm)
        self.get_logger().info(f"steering-pwm:{steer_pwm}...")

    def on_shutdown(self):
        self.set_pwm_us(ESC_CH, ESC_NEUTRAL_US)
        self.set_pwm_us(SERVO_CH, SERVO_CENTER_US)
        time.sleep(0.5)

# ================= MAIN =================
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

