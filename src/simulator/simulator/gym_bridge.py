#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
import numpy as np
from tf_transformations import quaternion_from_euler
import tf_transformations
from geometry_msgs.msg import Quaternion
import math
import numpy as np
# from simulator.sim_core import F1TenthSimulator  # adjust path if needed

class F1TenthSimulator:
    def __init__(self, map_path=None, map_ext=".png", num_agents=1, scan_fov=4.7, scan_beams=1080, timestep=0.01):
        # set instance variables for all arguments
        self.map_path = map_path
        self.map_ext = map_ext
        self.num_agents = num_agents
        self.scan_fov = scan_fov
        self.scan_beams = scan_beams
        self.timestep = timestep
        self.obs = {
            "poses_x": np.zeros(num_agents),
            "poses_y": np.zeros(num_agents),
            "poses_theta": np.zeros(num_agents),
            "linear_vels_x": np.zeros(num_agents),
            "linear_vels_y": np.zeros(num_agents),
            "ang_vels_z": np.zeros(num_agents),
            "scans": [np.ones(1080) * 10.0 for _ in range(num_agents)],
        }

    def reset(self):
        return self.obs

    def step(self, actions):
        import numpy as np
        steer, throttle = actions[0]
        for i in range(self.num_agents):
            self.obs["poses_theta"][i] += steer * 0.05
            self.obs["poses_x"][i] += np.cos(self.obs["poses_theta"][i]) * throttle * 0.1
            self.obs["poses_y"][i] += np.sin(self.obs["poses_theta"][i]) * throttle * 0.1
            self.obs["linear_vels_x"][i] = throttle
            self.obs["ang_vels_z"][i] = steer * 2.0
        return self.obs, 0.0, False, {}

    def render(self):
        pass


class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        # ===== Parameters =====
        self.declare_parameter('num_agents', 1)
        self.declare_parameter('map_path', '/home/goldenhawk/ros2_ws/RC-CAR-ROS2/src/simulator/maps/levine')
        self.declare_parameter('map_img_ext', '.png')
        self.declare_parameter('scan_fov', 4.7)
        self.declare_parameter('scan_beams', 1080)
        self.declare_parameter('kb_teleop', False)
        self.declare_parameter('scan_range', 10.0)
        self.num_agents = int(self.get_parameter('num_agents').value)
        self.actions = np.zeros((self.num_agents, 2))
        self.map_path = self.get_parameter('map_path').value
        self.map_ext = self.get_parameter('map_img_ext').value
        self.scan_fov = float(self.get_parameter('scan_fov').value)
        self.scan_beams = int(self.get_parameter('scan_beams').value)
        self.kb_teleop = bool(self.get_parameter('kb_teleop').value)

        self.has_opp = self.num_agents > 1

        # ===== Simulator =====
        self.env = F1TenthSimulator(
            map_path=self.map_path,
            map_ext=self.map_ext,
            num_agents=self.num_agents,
            scan_fov=self.scan_fov,
            scan_beams=self.scan_beams,
        )

        self.obs = self.env.reset()
        self.dt = 0.02

        # ===== Internal State =====
        self.speed_cmd = [0.0 for _ in range(self.num_agents)]
        self.steer_cmd = [0.0 for _ in range(self.num_agents)]

        # ===== ROS Setup =====
        self.br = TransformBroadcaster(self)

        self.scan_pubs = []
        self.odom_pubs = []
        for i in range(self.num_agents):
            ns = "ego_racecar" if i == 0 else "opp_racecar"
            self.scan_pubs.append(self.create_publisher(LaserScan, f'{ns}/scan', 10))
            self.odom_pubs.append(self.create_publisher(Odometry, f'{ns}/odom', 10))
            self.create_subscription(AckermannDriveStamped, f'{ns}/drive', lambda msg, i=i: self.drive_cb(msg, i), 10)
            self.create_subscription(
            AckermannDriveStamped,
            'ackermann_cmd',
            lambda msg: self.drive_cb(msg, 0),
            10
        )
        if self.kb_teleop:
            self.create_subscription(Twist, '/cmd_vel', self.teleop_cb, 10)

        # ===== Timers =====
        self.create_timer(self.dt, self.step_sim)

    def _safe_quat_from_yaw(self, yaw):
        """
        Converts a yaw angle to a geometry_msgs.msg.Quaternion safely.
        """
        # Ensure yaw is a standard float
        yaw = float(yaw)
        
        # Use tf_transformations to convert Euler (roll, pitch, yaw) to quaternion
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        
        # Create a Quaternion message
        quat_msg = Quaternion()
        quat_msg.x = q[0]
        quat_msg.y = q[1]
        quat_msg.z = q[2]
        quat_msg.w = q[3]
        return quat_msg
    

    # ===== Callbacks =====
    def drive_cb(self, msg, idx):
        print(f"Drive callback: idx={idx}, speed={msg.drive.speed}, steering={msg.drive.steering_angle}")
        self.speed_cmd[idx] = msg.drive.speed
        self.steer_cmd[idx] = msg.drive.steering_angle

    def teleop_cb(self, msg):
        self.speed_cmd[0] = msg.linear.x
        self.steer_cmd[0] = 0.5 * np.sign(msg.angular.z)

    # ===== Simulation Loop =====
    def step_sim(self):
        actions = np.column_stack((self.steer_cmd, self.speed_cmd))
        # self.obs = self.env.step(actions)
        # CORRECTED CODE in step_sim
        self.obs, self.reward, self.done, self.info = self.env.step(actions)
        self.truncated = self.info.get('TimeLimit.truncated', False)
        for i in range(self.num_agents):
            self.publish_scan(i)
            self.publish_odom(i)
            self.publish_tf(i)
            self.publish_wheel_transforms(i)

    # ===== Publishers =====
    def publish_scan(self, i):
        ns, base_frame, laser_frame, _ = self._ns_frames(i)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = laser_frame  # consistent with TF child frames
        scan.angle_min = -self.env.scan_fov / 2.0
        scan.angle_max = self.env.scan_fov / 2.0
        scan.angle_increment = self.env.scan_fov / float(self.env.scan_beams)
        scan.range_min = 0.0
        scan.range_max = 30.0
        # make sure obs has the right length and is numeric
        raw_ranges = self.obs['scans'][i]
        scan.ranges = [float(r) if not (r is None or np.isnan(r)) else scan.range_max for r in raw_ranges]
        try:
            self.scan_pubs[i].publish(scan)
        except Exception as e:
            self.get_logger().error(f"Failed to publish scan for {ns}: {e}")

    def publish_odom(self, i):
        ns, base_frame, laser_frame, child_frame = self._ns_frames(i)

        # read pose/vel and guard against NaNs
        x = float(self.obs['poses_x'][i])
        y = float(self.obs['poses_y'][i])
        theta = float(self.obs['poses_theta'][i])
        vx = float(self.obs['linear_vels_x'][i])
        vy = float(self.obs['linear_vels_y'][i])
        wz = float(self.obs['ang_vels_z'][i])

        if any(math.isnan(v) for v in (x, y, theta, vx, vy, wz)):
            self.get_logger().warn(f"NaN in odom data for {ns}, skipping publish")
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = child_frame  # MUST MATCH publish_tf child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y

        quat = self._safe_quat_from_yaw(theta)
        if quat is None:
            self.get_logger().warn(f"Invalid quaternion for {ns}, skipping odom publish")
            return
        # THE CORRECT CODE
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        try:
            self.odom_pubs[i].publish(odom)
        except Exception as e:
            self.get_logger().error(f"Failed to publish odom for {ns}: {e}")

    def _ns_frames(self, i):
        """
        Returns namespace and frame names for each agents (ego or opponent).

        Args:
            i (int): agents index (0 for ego, 1 for opponent)
        Returns:
            tuple: (namespace, base_frame, laser_frame, child_frame)
        """
        # Namespace: first car is ego, second is opponent
        ns = "ego_racecar" if i == 0 else "opp_racecar"

        # Frame names (must match URDF/xacro)
        base_frame = f"{ns}/base_link"
        laser_frame = f"{ns}/laser_model"

        # For TF publishing, we use base_link as the child frame
        child_frame = base_frame

        return ns, base_frame, laser_frame, child_frame

    def publish_wheel_transforms(self, i):
        """
        Publishes the TF for the front steering wheels.
        """
        ns, base_frame, laser_frame, child_frame = self._ns_frames(i)
        
        # Get the current steering angle for this agent
        steer_angle = float(self.steer_cmd[i])

        # Convert steer angle to quaternion
        quat = self._safe_quat_from_yaw(steer_angle)
        if quat is None:
            self.get_logger().warn(f"Invalid quaternion for {ns} wheels, skipping")
            return

        # Create a reusable TransformStamped message
        wheel_ts = TransformStamped()
        wheel_ts.header.stamp = self.get_clock().now().to_msg()
        wheel_ts.transform.rotation = quat
        
        # --- Publish Left Wheel Transform ---
        # Parent frame MUST match your URDF
        wheel_ts.header.frame_id = f"{ns}/front_left_hinge"  
        wheel_ts.child_frame_id = f"{ns}/front_left_wheel" 
        self.br.sendTransform(wheel_ts)

        # --- Publish Right Wheel Transform ---
        # Parent frame MUST match your URDF
        wheel_ts.header.frame_id = f"{ns}/front_right_hinge" 
        wheel_ts.child_frame_id = f"{ns}/front_right_wheel" 
        self.br.sendTransform(wheel_ts)

    def publish_tf(self, i):
        """
        Publishes the transform from map → base_link for each agents.
        """
        ns, base_frame, laser_frame, child_frame = self._ns_frames(i)

        # Extract pose info for this car
        x = float(self.obs['poses_x'][i])
        y = float(self.obs['poses_y'][i])
        theta = float(self.obs['poses_theta'][i])

        # Skip if data is invalid
        if any(math.isnan(v) for v in (x, y, theta)):
            self.get_logger().warn(f"NaN in TF data for {ns}, skipping transform")
            return

        # Convert yaw to quaternion
        quat = quaternion_from_euler(0.0, 0.0, theta)

        # Build the transform
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'map'           # parent frame (root)
        ts.child_frame_id = child_frame      # base_link (car)
        ts.transform.translation.x = x
        ts.transform.translation.y = y
        ts.transform.translation.z = 0.0
        ts.transform.rotation.x = quat[0]
        ts.transform.rotation.y = quat[1]
        ts.transform.rotation.z = quat[2]
        ts.transform.rotation.w = quat[3]

        # Publish the transform
        try:
            self.br.sendTransform(ts)
        except Exception as e:
            self.get_logger().error(f"Failed to send TF for {ns}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GymBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    