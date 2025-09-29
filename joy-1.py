import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import Adafruit_PCA9685

STEERING_CHANNEL = 0    # PCA9685 channel for steering servo
ENGINE_CHANNEL   = 1    # PCA9685 channel for rear wheel servo ("engine")

class RCCarDriver(Node):
    def __init__(self):
        super().__init__('rc_car_driver')
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)  # 50 Hz standard for servos

        # Subscribe to steering and engine command topics
        self.create_subscription(Float64, 'steering_cmd', self.steering_callback, 10)
        self.create_subscription(Float64, 'throttle_cmd', self.engine_callback, 10)

    def steering_callback(self, msg):
        # Map input -1.0(left) ... 0.0(center) ... +1.0(right) to PWM
        norm = max(-1.0, min(1.0, msg.data))
        min_pwm = 300    # full left
        mid_pwm = 400    # center
        max_pwm = 500    # full right
        pwm_value = int(mid_pwm + (max_pwm - mid_pwm) * norm) if norm > 0 else int(mid_pwm + (min_pwm - mid_pwm) * norm)
        self.pwm.set_pwm(STEERING_CHANNEL, 0, pwm_value)

    def engine_callback(self, msg):
        # Map -1.0(reverse) ... 0.0(stop) ... +1.0(forward) to PWM for rear wheel servo
        norm = max(-1.0, min(1.0, msg.data))
        min_pwm = 300    # full reverse
        mid_pwm = 400    # stop/neutral
        max_pwm = 500    # full forward
        pwm_value = int(mid_pwm + (max_pwm - mid_pwm) * norm) if norm > 0 else int(mid_pwm + (min_pwm - mid_pwm) * norm)
        self.pwm.set_pwm(ENGINE_CHANNEL, 0, pwm_value)

def main(args=None):
    rclpy.init(args=args)
    node = RCCarDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
