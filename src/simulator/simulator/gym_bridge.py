# MIT License

# ROS2 and Simulation related imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TransformStamped, Transform
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster
import gym
import numpy as np
from transforms3d import euler

class GymBridge(Node):
    """
    ROS2 Node acting as a bridge between the F1TENTH Gym simulation environment
    and ROS2 topics for a single race car.
    """
    def __init__(self):
        super().__init__('gym_bridge')

        # --- Declare and fetch parameters from ROS2 parameter server ---
        # Parameters for the single 'ego' car
        self.declare_parameter('ego_namespace', 'ego')
        self.declare_parameter('ego_odom_topic', 'odom')
        self.declare_parameter('ego_scan_topic', 'scan')
        self.declare_parameter('ego_drive_topic', 'drive')
        
        # Simulation and sensor parameters
        self.declare_parameter('scan_distance_to_base_link', 0.27)
        self.declare_parameter('scan_fov', 4.7)
        self.declare_parameter('scan_beams', 1080)
        self.declare_parameter('map_path')
        self.declare_parameter('map_img_ext', '.png')
        
        # Initial pose parameters for the car
        self.declare_parameter('sx', 0.0)
        self.declare_parameter('sy', 0.0)
        self.declare_parameter('stheta', 0.0)
        
        # Teleop parameter
        self.declare_parameter('kb_teleop', False)

        # --- Setup F1TENTH Gym environment for a single agent ---
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.get_parameter('map_path').value,
                            map_ext=self.get_parameter('map_img_ext').value,
                            num_agents=1)

        # --- Initialize car state variables ---
        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value
        self.ego_pose = [sx, sy, stheta]              # Car pose [x, y, theta]
        self.ego_speed = [0.0, 0.0, 0.0]             # Car speed [vx, vy, wz]
        self.ego_requested_speed = 0.0               # Commanded speed from ROS topic
        self.ego_steer = 0.0                         # Commanded steering angle

        # --- Reset simulation with the initial pose ---
        self.obs, _, self.done, _ = self.env.reset(np.array([[sx, sy, stheta]]))
        self.ego_scan = list(self.obs['scans'][0])

        # --- Configure topics and LiDAR scan parameters ---
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
        
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value

        # --- Timers for simulation and data publishing ---
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        self.timer = self.create_timer(0.004, self.timer_callback)

        # --- TF transform broadcaster ---
        self.br = TransformBroadcaster(self)

        # --- Publishers ---
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False

        # --- Subscribers ---
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped, ego_drive_topic, self.drive_callback, 10)
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.ego_reset_callback, 10)
        
        # Optional keyboard teleop subscriber
        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist, '/cmd_vel', self.teleop_callback, 10)

    # -- ROS Callbacks --

    def drive_callback(self, drive_msg):
        """Handle drive commands for the car."""
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True

    def ego_reset_callback(self, pose_msg):
        """Reset car pose in simulation based on a message from RViz."""
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        orientation = pose_msg.pose.pose.orientation
        _, _, rtheta = euler.quat2euler(
            [orientation.w, orientation.x, orientation.y, orientation.z], axes='sxyz')
        
        # Reset the environment with the new pose for the single car
        self.obs, _, self.done, _ = self.env.reset(np.array([[rx, ry, rtheta]]))

    def teleop_callback(self, twist_msg):
        """Allow controlling the car using keyboard commands (teleop)."""
        if not self.ego_drive_published:
            self.ego_drive_published = True
        self.ego_requested_speed = twist_msg.linear.x
        # Set steering based on angular direction
        self.ego_steer = 0.3 if twist_msg.angular.z > 0.0 else -0.3 if twist_msg.angular.z < 0.0 else 0.0

    # -- Timers and Publishing --

    def drive_timer_callback(self):
        """Step simulation forward using recent drive commands."""
        if self.ego_drive_published:
            action = np.array([[self.ego_steer, self.ego_requested_speed]])
            self.obs, _, self.done, _ = self.env.step(action)
        
        self._update_sim_state()

    def timer_callback(self):
        """Publish sensor data and TF transforms periodically."""
        ts = self.get_clock().now().to_msg()
        
        # Publish LaserScan
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = self.ego_namespace + '/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = 30.
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        # Publish odometry and transforms
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        """Update stored states from the simulator outputs after a step."""
        # Update scan and pose from the 'obs' dictionary
        self.ego_scan = list(self.obs['scans'][0])
        self.ego_pose[0] = self.obs['poses_x'][0]
        self.ego_pose[1] = self.obs['poses_y'][0]
        self.ego_pose[2] = self.obs['poses_theta'][0]
        self.ego_speed[0] = self.obs['linear_vels_x'][0]
        self.ego_speed[1] = self.obs['linear_vels_y'][0]
        self.ego_speed[2] = self.obs['ang_vels_z'][0]

    def _publish_odom(self, ts):
        """Publish the Odometry message for the car."""
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = 'map'
        ego_odom.child_frame_id = self.ego_namespace + '/base_link'
        
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        
        ego_quat = euler.euler2quat(0., 0., self.ego_pose[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = ego_quat[1]
        ego_odom.pose.pose.orientation.y = ego_quat[2]
        ego_odom.pose.pose.orientation.z = ego_quat[3]
        ego_odom.pose.pose.orientation.w = ego_quat[0]
        
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        
        self.ego_odom_pub.publish(ego_odom)

    def _publish_transforms(self, ts):
        """Publish the base_link TF transform for the vehicle."""
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        
        ego_quat = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        ego_t.rotation.x = ego_quat[1]
        ego_t.rotation.y = ego_quat[2]
        ego_t.rotation.z = ego_quat[3]
        ego_t.rotation.w = ego_quat[0]
        
        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = 'map'
        ego_ts.child_frame_id = self.ego_namespace + '/base_link'
        
        self.br.sendTransform(ego_ts)

    def _publish_wheel_transforms(self, ts):
        """Publish TF for front wheels steering."""
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = euler.euler2quat(0., 0., self.ego_steer, axes='sxyz')
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[3]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[0]
        ego_wheel_ts.header.stamp = ts
        
        # Left wheel
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)

        # Right wheel
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

    def _publish_laser_transforms(self, ts):
        """Publish the TF transform for the LIDAR scan frame."""
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        ego_scan_ts.transform.rotation.w = 1.0 # No rotation relative to base_link
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
        ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
        self.br.sendTransform(ego_scan_ts)

def main(args=None):
    """Entry point for running the ROS2 gym bridge node."""
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
