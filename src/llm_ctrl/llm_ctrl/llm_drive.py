import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header

from .llm_controller import parse_drive_command


class LlmAckermannNode(Node):
    def __init__(self):
        super().__init__("llm_ackermann_node")

        # Publisher: adjust topic name to match your car’s controller
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            "/cmd_ackermann",  # or "/ackermann_cmd", etc.
            10,
        )

        # For demo: a basic timer to read console input.
        # In production, replace with a real input source (websocket, etc.).
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.waiting_for_input = True

    def timer_callback(self):
        if not self.waiting_for_input:
            return

        try:
            # Blocking input in a timer is hacky but simple for a demo.
            self.waiting_for_input = False
            command = input("Enter drive command (e.g., 'set velocity to 2'): ")
        except EOFError:
            return

        drive_params = parse_drive_command(command)
        self.get_logger().info(f"Parsed drive params: {drive_params}")

        msg = AckermannDriveStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.drive.steering_angle = float(drive_params["steering_angle"])
        msg.drive.steering_angle_velocity = float(
            drive_params["steering_angle_velocity"]
        )
        msg.drive.speed = float(drive_params["speed"])
        msg.drive.acceleration = float(drive_params["acceleration"])
        msg.drive.jerk = float(drive_params["jerk"])

        self.pub.publish(msg)
        self.get_logger().info(
            f"Published AckermannDriveStamped: speed={msg.drive.speed}, steering_angle={msg.drive.steering_angle}"
        )

        # Allow another input on the next timer tick
        self.waiting_for_input = True


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
