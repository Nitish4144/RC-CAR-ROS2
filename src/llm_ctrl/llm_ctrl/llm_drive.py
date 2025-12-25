import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header
from rclpy.qos import QoSProfile

import threading

from .llm_controller import parse_drive_command


class LlmAckermannNode(Node):
    def __init__(self):
        super().__init__("llm_ackermann_node")

        # QoS for control commands
        qos = QoSProfile(depth=10)

        # Publisher
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            "/cmd_ackermann",
            qos,
        )

        # Safety limits (TUNE FOR YOUR CAR)
        self.MAX_SPEED = 3.0      # m/s
        self.MAX_STEER = 0.6      # rad
        self.MAX_ACCEL = 2.0      # m/s^2
        self.MAX_JERK = 5.0       # m/s^3

        # Start non-blocking input thread
        threading.Thread(target=self.input_loop, daemon=True).start()

        self.get_logger().info("LLM Ackermann node started. Waiting for commands...")

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    def input_loop(self):
        while rclpy.ok():
            try:
                command = input("Enter drive command: ")
            except EOFError:
                break

            self.publish_command(command)

    def publish_command(self, command: str):
        drive_params = parse_drive_command(command)
        self.get_logger().info(f"Parsed drive params: {drive_params}")

        msg = AckermannDriveStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Assign values
        msg.drive.speed = float(drive_params["speed"])
        msg.drive.steering_angle = float(drive_params["steering_angle"])
        msg.drive.steering_angle_velocity = float(
            drive_params["steering_angle_velocity"]
        )
        msg.drive.acceleration = float(drive_params["acceleration"])
        msg.drive.jerk = float(drive_params["jerk"])

        # --- SAFETY CLAMPING ---
        msg.drive.speed = self.clamp(
            msg.drive.speed, -self.MAX_SPEED, self.MAX_SPEED
        )
        msg.drive.steering_angle = self.clamp(
            msg.drive.steering_angle, -self.MAX_STEER, self.MAX_STEER
        )
        msg.drive.acceleration = self.clamp(
            msg.drive.acceleration, -self.MAX_ACCEL, self.MAX_ACCEL
        )
        msg.drive.jerk = self.clamp(
            msg.drive.jerk, -self.MAX_JERK, self.MAX_JERK
        )

        self.pub.publish(msg)

        self.get_logger().info(
            f"Published: speed={msg.drive.speed:.2f} m/s, "
            f"steer={msg.drive.steering_angle:.2f} rad"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LlmAckermannNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
