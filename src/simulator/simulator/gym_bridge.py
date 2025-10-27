# # MIT License

# # ROS2 and Simulation related imports
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TransformStamped, Transform
# from ackermann_msgs.msg import AckermannDriveStamped
# from tf2_ros import TransformBroadcaster
# import gymnasium as gym
# import numpy as np
# from transforms3d import euler

# class GymBridge(Node):
#     """
#     ROS2 Node acting as a bridge between the F110 Gym simulation environment
#     and ROS2 topics. Supports single or dual (opponent) cars.
#     """
#     def __init__(self):
#         super().__init__('gym_bridge')

#         # Declare and fetch parameters from ROS2 parameter server
#         self.declare_parameter('ego_namespace')
#         self.declare_parameter('ego_odom_topic')
#         self.declare_parameter('ego_opp_odom_topic')
#         self.declare_parameter('ego_scan_topic')
#         self.declare_parameter('ego_drive_topic')
#         self.declare_parameter('opp_namespace')
#         self.declare_parameter('opp_odom_topic')
#         self.declare_parameter('opp_ego_odom_topic')
#         self.declare_parameter('opp_scan_topic')
#         self.declare_parameter('opp_drive_topic')
#         self.declare_parameter('scan_distance_to_base_link')
#         self.declare_parameter('scan_fov')
#         self.declare_parameter('scan_beams')
#         self.declare_parameter('map_path')
#         self.declare_parameter('map_img_ext')
#         self.declare_parameter('num_agent')
#         self.declare_parameter('sx')
#         self.declare_parameter('sy')
#         self.declare_parameter('stheta')
#         self.declare_parameter('sx1')
#         self.declare_parameter('sy1')
#         self.declare_parameter('stheta1')
#         self.declare_parameter('kb_teleop')

#         # Handle multi-agent (ego/opponent) setup
#         # num_agents = self.get_parameter('num_agent').value
#         num_agents =1
#         if num_agents not in [1, 2] or not isinstance(num_agents, int):
#             raise ValueError('num_agents should be either 1 or 2 and int.')

#         # Setup F1Tenth Gym environment
#         self.env = gym.make('f110_gym:f110-v0',
#                             map=self.get_parameter('map_path').value,
#                             map_ext=self.get_parameter('map_img_ext').value,
#                             num_agents=num_agents)

#         # Initial positions and pose state
#         sx = self.get_parameter('sx').value
#         sy = self.get_parameter('sy').value
#         stheta = self.get_parameter('stheta').value
#         self.ego_pose = [sx, sy, stheta]                # Ego car pose (x, y, theta)
#         self.ego_speed = [0.0, 0.0, 0.0]                # Ego car speed (vx, vy, wz)
#         self.ego_requested_speed = 0.0                  # Commanded speed (from ROS)
#         self.ego_steer = 0.0                            # Commanded steering
#         self.ego_collision = False                      # Flag for collision event

#         # Topics and scan configuration
#         ego_scan_topic = self.get_parameter('ego_scan_topic').value
#         ego_drive_topic = self.get_parameter('ego_drive_topic').value
#         scan_fov = self.get_parameter('scan_fov').value
#         scan_beams = self.get_parameter('scan_beams').value
#         self.angle_min = -scan_fov / 2.
#         self.angle_max = scan_fov / 2.
#         self.angle_inc = scan_fov / scan_beams
#         self.ego_namespace = self.get_parameter('ego_namespace').value
#         ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
#         self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value

#         # Opponent (second car) setup if present
#         if num_agents == 2:
#             self.has_opp = True
#             self.opp_namespace = self.get_parameter('opp_namespace').value
#             sx1 = self.get_parameter('sx1').value
#             sy1 = self.get_parameter('sy1').value
#             stheta1 = self.get_parameter('stheta1').value
#             self.opp_pose = [sx1, sy1, stheta1]
#             self.opp_speed = [0.0, 0.0, 0.0]
#             self.opp_requested_speed = 0.0
#             self.opp_steer = 0.0
#             self.opp_collision = False
#             # Reset environment with both agents' poses
#             self.obs, _, self.done, _ = self.env.reset(np.array([[sx, sy, stheta], [sx1, sy1, stheta1]]))
#             self.ego_scan = list(self.obs['scans'][0])
#             self.opp_scan = list(self.obs['scans'][1])
#             opp_scan_topic = self.get_parameter('opp_scan_topic').value
#             opp_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_odom_topic').value
#             opp_drive_topic = self.get_parameter('opp_drive_topic').value
#             ego_opp_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_opp_odom_topic').value
#             opp_ego_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_ego_odom_topic').value
#         else:
#             self.has_opp = False
#             # Reset environment with only ego agent
#             self.obs, _, self.done, _ = self.env.reset(np.array([[sx, sy, stheta]]))
#             self.ego_scan = list(self.obs['scans'][0])

#         # Timers for stepping sim and data publishing
#         self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
#         self.timer = self.create_timer(0.004, self.timer_callback)

#         # TF transform broadcaster
#         self.br = TransformBroadcaster(self)

#         # Publishers for ego and (optionally) opponent topics
#         self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
#         self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
#         self.ego_drive_published = False

#         if self.has_opp:
#             self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
#             self.ego_opp_odom_pub = self.create_publisher(Odometry, ego_opp_odom_topic, 10)
#             self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
#             self.opp_ego_odom_pub = self.create_publisher(Odometry, opp_ego_odom_topic, 10)
#             self.opp_drive_published = False

#         # Subscribers for drive commands, resets, and teleop (keyboard control)
#         # self.ego_drive_sub = self.create_subscription(
#         #     AckermannDriveStamped, ego_drive_topic, self.drive_callback, 10)
#         # Ego drive subscription (existing)
#         self.ego_drive_sub = self.create_subscription(
#             AckermannDriveStamped, ego_drive_topic, self.drive_callback, 10)

#         # Additional drive subscription (e.g., from joystick node)
#         self.joy_drive_sub = self.create_subscription(
#             AckermannDriveStamped, 'ackermann_cmd', self.joy_drive_callback, 10)

#         self.ego_reset_sub = self.create_subscription(
#             PoseWithCovarianceStamped, '/initialpose', self.ego_reset_callback, 10)
#         if self.has_opp:
#             self.opp_drive_sub = self.create_subscription(
#                 AckermannDriveStamped, opp_drive_topic, self.opp_drive_callback, 10)
#             self.opp_reset_sub = self.create_subscription(
#                 PoseStamped, '/goal_pose', self.opp_reset_callback, 10)
#         if self.get_parameter('kb_teleop').value:
#             # Keyboard teleop
#             self.teleop_sub = self.create_subscription(
#                 Twist, '/cmd_vel', self.teleop_callback, 10)

#     # -- ROS Callbacks --

#     def drive_callback(self, drive_msg):
#         """Handle drive commands for the ego car."""
#         self.ego_requested_speed = drive_msg.drive.speed
#         self.ego_steer = drive_msg.drive.steering_angle
#         self.ego_drive_published = True

#     def opp_drive_callback(self, drive_msg):
#         """Handle drive commands for opponent car (if present)."""
#         self.opp_requested_speed = drive_msg.drive.speed
#         self.opp_steer = drive_msg.drive.steering_angle
#         self.opp_drive_published = True

#     def ego_reset_callback(self, pose_msg):
#         """Reset ego car pose in simulation based on message."""
#         rx = pose_msg.pose.pose.position.x
#         ry = pose_msg.pose.pose.position.y
#         orientation = pose_msg.pose.pose.orientation
#         _, _, rtheta = euler.quat2euler(
#             [orientation.w, orientation.x, orientation.y, orientation.z], axes='sxyz')
#         # If opponent present, keep its pose, reset ego only
#         if self.has_opp:
#             opp_pose = [self.obs['poses_x'][1], self.obs['poses_y'][1], self.obs['poses_theta'][1]]
#             self.obs, _, self.done, _ = self.env.reset(np.array([[rx, ry, rtheta], opp_pose]))
#         else:
#             self.obs, _, self.done, _ = self.env.reset(np.array([[rx, ry, rtheta]]))

#     def opp_reset_callback(self, pose_msg):
#         """Reset opponent car pose in simulation (if 2 agents)."""
#         if self.has_opp:
#             rx = pose_msg.pose.position.x
#             ry = pose_msg.pose.position.y
#             orientation = pose_msg.pose.orientation
#             _, _, rtheta = euler.quat2euler(
#                 [orientation.w, orientation.x, orientation.y, orientation.z], axes='sxyz')
#             self.obs, _, self.done, _ = self.env.reset(np.array([list(self.ego_pose), [rx, ry, rtheta]]))

#     def teleop_callback(self, twist_msg):
#         """Allow controlling ego car using keyboard commands (teleop)."""
#         if not self.ego_drive_published:
#             self.ego_drive_published = True
#         self.ego_requested_speed = twist_msg.linear.x
#         # Set steering based on angular direction
#         self.ego_steer = 0.3 if twist_msg.angular.z > 0.0 else -0.3 if twist_msg.angular.z < 0.0 else 0.0

#     # -- Timers and Publishing --

#     def drive_timer_callback(self):
#         """Step simulation forward using recent drive commands."""
#         if self.ego_drive_published and not self.has_opp:
#             self.obs, _, self.done, _ = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed]]))
#         elif self.ego_drive_published and self.has_opp and self.opp_drive_published:
#             self.obs, _, self.done, _ = self.env.step(np.array([
#                 [self.ego_steer, self.ego_requested_speed],
#                 [self.opp_steer, self.opp_requested_speed]
#             ]))
#         self._update_sim_state()

#     def timer_callback(self):
#         """Publish sensor data and TF transforms periodically."""
#         ts = self.get_clock().now().to_msg()
#         # Publish ego scan
#         scan = LaserScan()
#         scan.header.stamp = ts
#         scan.header.frame_id = self.ego_namespace + '/laser'
#         scan.angle_min = self.angle_min
#         scan.angle_max = self.angle_max
#         scan.angle_increment = self.angle_inc
#         scan.range_min = 0.
#         scan.range_max = 30.
#         scan.ranges = self.ego_scan
#         self.ego_scan_pub.publish(scan)
#         # Publish opponent scan if present
#         if self.has_opp:
#             opp_scan = LaserScan()
#             opp_scan.header.stamp = ts
#             opp_scan.header.frame_id = self.opp_namespace + '/laser'
#             opp_scan.angle_min = self.angle_min
#             opp_scan.angle_max = self.angle_max
#             opp_scan.angle_increment = self.angle_inc
#             opp_scan.range_min = 0.
#             opp_scan.range_max = 30.
#             opp_scan.ranges = self.opp_scan
#             self.opp_scan_pub.publish(opp_scan)
#         # Publish odometry and transforms
#         self._publish_odom(ts)
#         self._publish_transforms(ts)
#         self._publish_laser_transforms(ts)
#         self._publish_wheel_transforms(ts)

#     def _update_sim_state(self):
#         """Update stored states from the simulator outputs after a step."""
#         self.ego_scan = list(self.obs['scans'][0])
#         # Update opponent if present
#         if self.has_opp:
#             self.opp_scan = list(self.obs['scans'][1])
#             self.opp_pose[0] = self.obs['poses_x'][1]
#             self.opp_pose[1] = self.obs['poses_y'][1]
#             self.opp_pose[2] = self.obs['poses_theta'][1]
#             self.opp_speed[0] = self.obs['linear_vels_x'][1]
#             self.opp_speed[1] = self.obs['linear_vels_y'][1]
#             self.opp_speed[2] = self.obs['ang_vels_z'][1]
#         # Ego state
#         self.ego_pose[0] = self.obs['poses_x'][0]
#         self.ego_pose[1] = self.obs['poses_y'][0]
#         self.ego_pose[2] = self.obs['poses_theta'][0]
#         self.ego_speed[0] = self.obs['linear_vels_x'][0]
#         self.ego_speed[1] = self.obs['linear_vels_y'][0]
#         self.ego_speed[2] = self.obs['ang_vels_z'][0]

#     def _publish_odom(self, ts):
#         """Publish Odometry messages for ego and opponent cars."""
#         # Ego odom
#         ego_odom = Odometry()
#         ego_odom.header.stamp = ts
#         ego_odom.header.frame_id = 'map'
#         ego_odom.child_frame_id = self.ego_namespace + '/base_link'
#         ego_odom.pose.pose.position.x = self.ego_pose[0]
#         ego_odom.pose.pose.position.y = self.ego_pose[1]
#         ego_quat = euler.euler2quat(0., 0., self.ego_pose[2], axes='sxyz')
#         ego_odom.pose.pose.orientation.x = ego_quat[1]
#         ego_odom.pose.pose.orientation.y = ego_quat[2]
#         ego_odom.pose.pose.orientation.z = ego_quat[3]
#         ego_odom.pose.pose.orientation.w = ego_quat[0]
#         ego_odom.twist.twist.linear.x = self.ego_speed[0]
#         ego_odom.twist.twist.linear.y = self.ego_speed[1]
#         ego_odom.twist.twist.angular.z = self.ego_speed[2]
#         self.ego_odom_pub.publish(ego_odom)
#         # Publish opponent odom and cross-odom (for applications needing both car data)
#         if self.has_opp:
#             opp_odom = Odometry()
#             opp_odom.header.stamp = ts
#             opp_odom.header.frame_id = 'map'
#             opp_odom.child_frame_id = self.opp_namespace + '/base_link'
#             opp_odom.pose.pose.position.x = self.opp_pose[0]
#             opp_odom.pose.pose.position.y = self.opp_pose[1]
#             opp_quat = euler.euler2quat(0., 0., self.opp_pose[2], axes='sxyz')
#             opp_odom.pose.pose.orientation.x = opp_quat[1]
#             opp_odom.pose.pose.orientation.y = opp_quat[2]
#             opp_odom.pose.pose.orientation.z = opp_quat[3]
#             opp_odom.pose.pose.orientation.w = opp_quat[0]
#             opp_odom.twist.twist.linear.x = self.opp_speed[0]
#             opp_odom.twist.twist.linear.y = self.opp_speed[1]
#             opp_odom.twist.twist.angular.z = self.opp_speed[2]
#             self.opp_odom_pub.publish(opp_odom)
#             self.opp_ego_odom_pub.publish(ego_odom)
#             self.ego_opp_odom_pub.publish(opp_odom)

#     def _publish_transforms(self, ts):
#         """Publish base_link TF transforms for both vehicles."""
#         # Ego transform
#         ego_t = Transform()
#         ego_t.translation.x = self.ego_pose[0]
#         ego_t.translation.y = self.ego_pose[1]
#         ego_t.translation.z = 0.0
#         ego_quat = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
#         ego_t.rotation.x = ego_quat[1]
#         ego_t.rotation.y = ego_quat[2]
#         ego_t.rotation.z = ego_quat[3]
#         ego_t.rotation.w = ego_quat[0]
#         ego_ts = TransformStamped()
#         ego_ts.transform = ego_t
#         ego_ts.header.stamp = ts
#         ego_ts.header.frame_id = 'map'
#         ego_ts.child_frame_id = self.ego_namespace + '/base_link'
#         self.br.sendTransform(ego_ts)
#         # Opponent transform if present
#         if self.has_opp:
#             opp_t = Transform()
#             opp_t.translation.x = self.opp_pose[0]
#             opp_t.translation.y = self.opp_pose[1]
#             opp_t.translation.z = 0.0
#             opp_quat = euler.euler2quat(0.0, 0.0, self.opp_pose[2], axes='sxyz')
#             opp_t.rotation.x = opp_quat[1]
#             opp_t.rotation.y = opp_quat[2]
#             opp_t.rotation.z = opp_quat[3]
#             opp_t.rotation.w = opp_quat[0]
#             opp_ts = TransformStamped()
#             opp_ts.transform = opp_t
#             opp_ts.header.stamp = ts
#             opp_ts.header.frame_id = 'map'
#             opp_ts.child_frame_id = self.opp_namespace + '/base_link'
#             self.br.sendTransform(opp_ts)

#     def _publish_wheel_transforms(self, ts):
#         """Publish TF for front wheels steering (for both cars)."""
#         # Ego left and right front wheel hinges
#         ego_wheel_ts = TransformStamped()
#         ego_wheel_quat = euler.euler2quat(0., 0., self.ego_steer, axes='sxyz')
#         ego_wheel_ts.transform.rotation.x = ego_wheel_quat[1]
#         ego_wheel_ts.transform.rotation.y = ego_wheel_quat[2]
#         ego_wheel_ts.transform.rotation.z = ego_wheel_quat[3]
#         ego_wheel_ts.transform.rotation.w = ego_wheel_quat[0]
#         ego_wheel_ts.header.stamp = ts
#         ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
#         ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
#         self.br.sendTransform(ego_wheel_ts)

#         ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
#         ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
#         self.br.sendTransform(ego_wheel_ts)

#         # Opponent wheels if present
#         if self.has_opp:
#             opp_wheel_ts = TransformStamped()
#             opp_wheel_quat = euler.euler2quat(0., 0., self.opp_steer, axes='sxyz')
#             opp_wheel_ts.transform.rotation.x = opp_wheel_quat[1]
#             opp_wheel_ts.transform.rotation.y = opp_wheel_quat[2]
#             opp_wheel_ts.transform.rotation.z = opp_wheel_quat[3]
#             opp_wheel_ts.transform.rotation.w = opp_wheel_quat[0]
#             opp_wheel_ts.header.stamp = ts
#             opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_left_hinge'
#             opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
#             self.br.sendTransform(opp_wheel_ts)

#             opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
#             opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
#             self.br.sendTransform(opp_wheel_ts)

#     def _publish_laser_transforms(self, ts):
#         """Publish TF transform for LIDAR scan frame (for both cars)."""
#         ego_scan_ts = TransformStamped()
#         ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
#         ego_scan_ts.transform.rotation.w = 1.
#         ego_scan_ts.header.stamp = ts
#         ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
#         ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
#         self.br.sendTransform(ego_scan_ts)
#         # Opponent LIDAR frame
#         if self.has_opp:
#             opp_scan_ts = TransformStamped()
#             opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
#             opp_scan_ts.transform.rotation.w = 1.
#             opp_scan_ts.header.stamp = ts
#             opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
#             opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
#             self.br.sendTransform(opp_scan_ts)

# def main(args=None):
#     """Entry point for running the ROS2 gym bridge node."""
#     rclpy.init(args=args)
#     gym_bridge = GymBridge()
#     rclpy.spin(gym_bridge)

# if __name__ == '__main__':
#     main()





# # MIT License

# # ROS2 and Simulation related imports
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TransformStamped, Transform
# from ackermann_msgs.msg import AckermannDriveStamped
# from tf2_ros import TransformBroadcaster
# import numpy as np
# from scipy.spatial.transform import Rotation
# import yaml
# from PIL import Image
# from numba import njit


# class VehicleDynamics:
#     """
#     Single-track (bicycle) vehicle dynamics model with slip for F1/10 racing
#     """
#     def __init__(self, params=None):
#         # Default F1TENTH vehicle parameters
#         self.params = {
#             'mu': 1.0489,           # friction coefficient
#             'C_Sf': 4.718,          # front cornering stiffness
#             'C_Sr': 5.4562,         # rear cornering stiffness
#             'lf': 0.15875,          # distance from CoG to front axle
#             'lr': 0.17145,          # distance from CoG to rear axle
#             'h': 0.074,             # CoG height
#             'm': 3.74,              # vehicle mass
#             'I': 0.04712,           # moment of inertia
#             's_min': -0.4189,       # minimum steering angle
#             's_max': 0.4189,        # maximum steering angle
#             'sv_min': -3.2,         # minimum steering velocity
#             'sv_max': 3.2,          # maximum steering velocity
#             'v_switch': 7.319,      # velocity switching threshold
#             'a_max': 9.51,          # maximum acceleration
#             'v_min': -5.0,          # minimum velocity
#             'v_max': 20.0,          # maximum velocity
#             'width': 0.31,          # vehicle width
#             'length': 0.58          # vehicle length
#         }
#         if params:
#             self.params.update(params)
        
#         self.wheelbase = self.params['lf'] + self.params['lr']
    
#     def step(self, state, action, dt):
#         """
#         Single integration step using single-track dynamics
#         state: [x, y, theta, velocity, steering_angle, angular_velocity, slip_angle]
#         action: [steering_velocity, acceleration]
#         """
#         x, y, theta, v, delta, omega, beta = state
#         sv, a = action
        
#         # Clip actions to limits
#         sv = np.clip(sv, self.params['sv_min'], self.params['sv_max'])
#         a = np.clip(a, -self.params['a_max'], self.params['a_max'])
        
#         # Update steering angle
#         delta += sv * dt
#         delta = np.clip(delta, self.params['s_min'], self.params['s_max'])
        
#         # Update velocity
#         v += a * dt
#         v = np.clip(v, self.params['v_min'], self.params['v_max'])
        
#         # Compute slip angle (simplified)
#         if abs(v) > 0.1:
#             beta = np.arctan(self.params['lr'] * np.tan(delta) / self.wheelbase)
#         else:
#             beta = 0.0
        
#         # Update angular velocity
#         if abs(v) > 0.1:
#             omega = v * np.cos(beta) * np.tan(delta) / self.wheelbase
#         else:
#             omega = 0.0
        
#         # Update position and heading
#         x += v * np.cos(theta + beta) * dt
#         y += v * np.sin(theta + beta) * dt
#         theta += omega * dt
        
#         # Normalize theta to [-pi, pi]
#         theta = np.arctan2(np.sin(theta), np.cos(theta))
        
#         return np.array([x, y, theta, v, delta, omega, beta])


# class RaceTrack:
#     """
#     Race track representation with collision detection and LIDAR simulation
#     """
#     def __init__(self, map_path, map_ext='.png', scan_beams=1080, scan_fov=4.7):
#         self.scan_beams = scan_beams
#         self.scan_fov = scan_fov
#         self.scan_angles = np.linspace(-scan_fov/2, scan_fov/2, scan_beams)
        
#         # Load map
#         yaml_path = map_path + '.yaml' if not map_path.endswith('.yaml') else map_path
#         try:
#             with open(yaml_path, 'r') as f:
#                 map_config = yaml.safe_load(f)
#         except:
#             # Default values if yaml loading fails
#             map_config = {
#                 'resolution': 0.05,
#                 'origin': [0.0, 0.0, 0.0]
#             }
        
#         self.resolution = map_config.get('resolution', 0.05)
#         self.origin = map_config.get('origin', [0.0, 0.0, 0.0])
        
#         # Load map image
#         img_path = map_path.replace('.yaml', map_ext)
#         try:
#             img = Image.open(img_path).convert('L')
#             self.map_img = np.array(img)
#         except:
#             # Create a simple rectangular track if image loading fails
#             self.map_img = self._create_default_track()
        
#         self.height, self.width = self.map_img.shape
        
#         # Create occupancy grid (0 = free, 1 = occupied)
#         self.occupancy = (self.map_img < 128).astype(np.uint8)
    
#     def _create_default_track(self):
#         """Create a simple rectangular track for testing"""
#         track = np.ones((500, 500), dtype=np.uint8) * 255
#         # Draw outer boundary
#         track[50:450, 50:450] = 128
#         # Draw inner boundary
#         track[150:350, 150:350] = 255
#         return track
    
#     def world_to_map(self, x, y):
#         """Convert world coordinates to map pixel coordinates"""
#         map_x = int((x - self.origin[0]) / self.resolution)
#         map_y = int((y - self.origin[1]) / self.resolution)
#         return map_x, map_y
    
#     def is_collision(self, x, y):
#         """Check if position collides with track boundaries"""
#         map_x, map_y = self.world_to_map(x, y)
        
#         if map_x < 0 or map_x >= self.width or map_y < 0 or map_y >= self.height:
#             return True
        
#         return self.occupancy[map_y, map_x] == 1
    
#     def scan(self, x, y, theta, max_range=30.0):
#         """
#         Simulate LIDAR scan from given pose
#         Returns array of ranges for each beam
#         """
#         ranges = []
        
#         for angle in self.scan_angles:
#             beam_angle = theta + angle
            
#             # Ray casting
#             current_range = 0.0
#             step = 0.05  # Step size in meters
            
#             while current_range < max_range:
#                 current_range += step
                
#                 # Calculate point along ray
#                 ray_x = x + current_range * np.cos(beam_angle)
#                 ray_y = y + current_range * np.sin(beam_angle)
                
#                 # Check if hit obstacle
#                 if self.is_collision(ray_x, ray_y):
#                     ranges.append(current_range)
#                     break
#             else:
#                 # Max range reached
#                 ranges.append(max_range)
        
#         return np.array(ranges)


# class F1TenthSimulator:
#     """
#     Complete F1TENTH racing simulator
#     """
#     def __init__(self, map_path, map_ext='.png', num_agents=1, scan_beams=1080, scan_fov=4.7):
#         self.num_agents = num_agents
#         self.track = RaceTrack(map_path, map_ext, scan_beams, scan_fov)
        
#         # Initialize vehicle dynamics for each agent
#         self.vehicles = [VehicleDynamics() for _ in range(num_agents)]
        
#         # States for each vehicle [x, y, theta, v, delta, omega, beta]
#         self.states = [np.zeros(7) for _ in range(num_agents)]
        
#         # Collision flags
#         self.collisions = [False] * num_agents
        
#         self.dt = 0.01  # 100 Hz simulation
    
#     def reset(self, poses):
#         """
#         Reset simulation with initial poses
#         poses: array of shape (num_agents, 3) containing [x, y, theta]
#         """
#         for i, pose in enumerate(poses):
#             self.states[i] = np.array([
#                 pose[0],  # x
#                 pose[1],  # y
#                 pose[2],  # theta
#                 0.0,      # velocity
#                 0.0,      # steering angle
#                 0.0,      # angular velocity
#                 0.0       # slip angle
#             ])
#             self.collisions[i] = False
        
#         return self._get_observation()
    
#     def step(self, actions):
#         """
#         Step simulation forward
#         actions: array of shape (num_agents, 2) containing [steering_cmd, speed_cmd]
#         """
#         for i in range(self.num_agents):
#             if not self.collisions[i]:
#                 # Convert high-level command to dynamics input
#                 steer_cmd, speed_cmd = actions[i]
                
#                 # Calculate acceleration to reach desired speed
#                 current_speed = self.states[i][3]
#                 acceleration = (speed_cmd - current_speed) * 5.0  # P controller
                
#                 # Steering velocity (P controller)
#                 current_steer = self.states[i][4]
#                 steering_velocity = (steer_cmd - current_steer) * 3.0
                
#                 # Step dynamics
#                 self.states[i] = self.vehicles[i].step(
#                     self.states[i],
#                     [steering_velocity, acceleration],
#                     self.dt
#                 )
                
#                 # Check collision
#                 x, y = self.states[i][0], self.states[i][1]
#                 if self.track.is_collision(x, y):
#                     self.collisions[i] = True
        
#         return self._get_observation()
    
#     def _get_observation(self):
#         """Generate observation dictionary"""
#         obs = {
#             'scans': [],
#             'poses_x': [],
#             'poses_y': [],
#             'poses_theta': [],
#             'linear_vels_x': [],
#             'linear_vels_y': [],
#             'ang_vels_z': [],
#             'collisions': []
#         }
        
#         for i in range(self.num_agents):
#             x, y, theta, v, delta, omega, beta = self.states[i]
            
#             # Generate scan
#             scan = self.track.scan(x, y, theta)
#             obs['scans'].append(scan)
            
#             # Add state information
#             obs['poses_x'].append(x)
#             obs['poses_y'].append(y)
#             obs['poses_theta'].append(theta)
#             obs['linear_vels_x'].append(v * np.cos(beta))
#             obs['linear_vels_y'].append(v * np.sin(beta))
#             obs['ang_vels_z'].append(omega)
#             obs['collisions'].append(float(self.collisions[i]))
        
#         return obs


# class GymBridge(Node):
#     """
#     ROS2 Node acting as a bridge between the custom F1TENTH simulation
#     and ROS2 topics. Supports single or dual (opponent) cars.
#     """
#     def __init__(self):
#         super().__init__('gym_bridge')

#         # Declare and fetch parameters
#         self.declare_parameter('ego_namespace', 'ego_racecar')
#         self.declare_parameter('ego_odom_topic', 'odom')
#         self.declare_parameter('ego_opp_odom_topic', 'opp_odom')
#         self.declare_parameter('ego_scan_topic', 'scan')
#         self.declare_parameter('ego_drive_topic', 'drive')
#         self.declare_parameter('opp_namespace', 'opp_racecar')
#         self.declare_parameter('opp_odom_topic', 'odom')
#         self.declare_parameter('opp_ego_odom_topic', 'ego_odom')
#         self.declare_parameter('opp_scan_topic', 'scan')
#         self.declare_parameter('opp_drive_topic', 'drive')
#         self.declare_parameter('scan_distance_to_base_link', 0.275)
#         self.declare_parameter('scan_fov', 4.7)
#         self.declare_parameter('scan_beams', 1080)
#         self.declare_parameter('map_path', '/path/to/map')
#         self.declare_parameter('map_img_ext', '.png')
#         self.declare_parameter('num_agent', 1)
#         self.declare_parameter('sx', 0.0)
#         self.declare_parameter('sy', 0.0)
#         self.declare_parameter('stheta', 0.0)
#         self.declare_parameter('sx1', 2.0)
#         self.declare_parameter('sy1', 0.0)
#         self.declare_parameter('stheta1', 0.0)
#         self.declare_parameter('kb_teleop', False)

#         # Get parameters
#         num_agents = self.get_parameter('num_agent').value
#         if num_agents not in [1, 2]:
#             raise ValueError('num_agents should be either 1 or 2')

#         # Initialize simulator
#         map_path = self.get_parameter('map_path').value
#         # map_path = '/home/goldenhawk/ros2_ws/RC-CAR-ROS2/src/simulator/maps/levine'  # Example hardcoded path
#         map_ext = self.get_parameter('map_img_ext').value
#         scan_fov = self.get_parameter('scan_fov').value
#         scan_beams = self.get_parameter('scan_beams').value
        
#         self.env = F1TenthSimulator(
#             map_path=map_path,
#             map_ext=map_ext,
#             num_agents=num_agents,
#             scan_beams=scan_beams,
#             scan_fov=scan_fov
#         )

#         # Initial poses
#         sx = self.get_parameter('sx').value
#         sy = self.get_parameter('sy').value
#         stheta = self.get_parameter('stheta').value
#         self.ego_pose = [sx, sy, stheta]
#         self.ego_speed = [0.0, 0.0, 0.0]
#         self.ego_requested_speed = 0.0
#         self.ego_steer = 0.0
#         self.ego_collision = False

#         # Scan configuration
#         self.angle_min = -scan_fov / 2.
#         self.angle_max = scan_fov / 2.
#         self.angle_inc = scan_fov / scan_beams
#         self.ego_namespace = self.get_parameter('ego_namespace').value
#         ego_scan_topic = self.get_parameter('ego_scan_topic').value
#         ego_drive_topic = self.get_parameter('ego_drive_topic').value
#         ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
#         self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value

#         # Opponent setup
#         if num_agents == 2:
#             self.has_opp = True
#             self.opp_namespace = self.get_parameter('opp_namespace').value
#             sx1 = self.get_parameter('sx1').value
#             sy1 = self.get_parameter('sy1').value
#             stheta1 = self.get_parameter('stheta1').value
#             self.opp_pose = [sx1, sy1, stheta1]
#             self.opp_speed = [0.0, 0.0, 0.0]
#             self.opp_requested_speed = 0.0
#             self.opp_steer = 0.0
#             self.opp_collision = False
            
#             # Reset with both agents
#             self.obs = self.env.reset(np.array([[sx, sy, stheta], [sx1, sy1, stheta1]]))
#             self.ego_scan = list(self.obs['scans'][0])
#             self.opp_scan = list(self.obs['scans'][1])
            
#             opp_scan_topic = self.get_parameter('opp_scan_topic').value
#             opp_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_odom_topic').value
#             opp_drive_topic = self.get_parameter('opp_drive_topic').value
#             ego_opp_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_opp_odom_topic').value
#             opp_ego_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_ego_odom_topic').value
#         else:
#             self.has_opp = False
#             self.obs = self.env.reset(np.array([[sx, sy, stheta]]))
#             self.ego_scan = list(self.obs['scans'][0])

#         # Timers
#         self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
#         self.timer = self.create_timer(0.004, self.timer_callback)

#         # TF broadcaster
#         self.br = TransformBroadcaster(self)

#         # Publishers
#         self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
#         self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
#         self.ego_drive_published = False

#         if self.has_opp:
#             self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
#             self.ego_opp_odom_pub = self.create_publisher(Odometry, ego_opp_odom_topic, 10)
#             self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
#             self.opp_ego_odom_pub = self.create_publisher(Odometry, opp_ego_odom_topic, 10)
#             self.opp_drive_published = False

#         # Subscribers
#         self.ego_drive_sub = self.create_subscription(
#             AckermannDriveStamped, ego_drive_topic, self.drive_callback, 10)
        
#         self.joy_drive_sub = self.create_subscription(
#             AckermannDriveStamped, 'ackermann_cmd', self.joy_drive_callback, 10)
        
#         self.ego_reset_sub = self.create_subscription(
#             PoseWithCovarianceStamped, '/initialpose', self.ego_reset_callback, 10)
        
#         if self.has_opp:
#             self.opp_drive_sub = self.create_subscription(
#                 AckermannDriveStamped, opp_drive_topic, self.opp_drive_callback, 10)
#             self.opp_reset_sub = self.create_subscription(
#                 PoseStamped, '/goal_pose', self.opp_reset_callback, 10)
        
#         if self.get_parameter('kb_teleop').value:
#             self.teleop_sub = self.create_subscription(
#                 Twist, '/cmd_vel', self.teleop_callback, 10)
        
#         self.get_logger().info('F1TENTH Gym Bridge initialized successfully')

#     def euler_to_quaternion(self, roll, pitch, yaw):
#         """Convert euler angles to quaternion using scipy"""
#         r = Rotation.from_euler('xyz', [roll, pitch, yaw])
#         quat = r.as_quat()  # Returns [x, y, z, w]
#         return quat

#     def quaternion_to_euler(self, x, y, z, w):
#         """Convert quaternion to euler angles using scipy"""
#         r = Rotation.from_quat([x, y, z, w])
#         euler = r.as_euler('xyz')  # Returns [roll, pitch, yaw]
#         return euler

#     # ROS Callbacks
#     def drive_callback(self, drive_msg):
#         """Handle drive commands for ego car"""
#         self.ego_requested_speed = drive_msg.drive.speed
#         self.ego_steer = drive_msg.drive.steering_angle
#         self.ego_drive_published = True

#     def joy_drive_callback(self, drive_msg):
#         """Handle joystick drive commands"""
#         self.ego_requested_speed = drive_msg.drive.speed
#         self.ego_steer = drive_msg.drive.steering_angle
#         self.ego_drive_published = True

#     def opp_drive_callback(self, drive_msg):
#         """Handle drive commands for opponent"""
#         self.opp_requested_speed = drive_msg.drive.speed
#         self.opp_steer = drive_msg.drive.steering_angle
#         self.opp_drive_published = True

#     def ego_reset_callback(self, pose_msg):
#         """Reset ego car pose"""
#         rx = pose_msg.pose.pose.position.x
#         ry = pose_msg.pose.pose.position.y
#         orientation = pose_msg.pose.pose.orientation
#         _, _, rtheta = self.quaternion_to_euler(
#             orientation.x, orientation.y, orientation.z, orientation.w)
        
#         if self.has_opp:
#             opp_pose = [self.obs['poses_x'][1], self.obs['poses_y'][1], self.obs['poses_theta'][1]]
#             self.obs = self.env.reset(np.array([[rx, ry, rtheta], opp_pose]))
#         else:
#             self.obs = self.env.reset(np.array([[rx, ry, rtheta]]))

#     def opp_reset_callback(self, pose_msg):
#         """Reset opponent pose"""
#         if self.has_opp:
#             rx = pose_msg.pose.position.x
#             ry = pose_msg.pose.position.y
#             orientation = pose_msg.pose.orientation
#             _, _, rtheta = self.quaternion_to_euler(
#                 orientation.x, orientation.y, orientation.z, orientation.w)
#             self.obs = self.env.reset(np.array([list(self.ego_pose), [rx, ry, rtheta]]))

#     def teleop_callback(self, twist_msg):
#         """Keyboard teleop control"""
#         if not self.ego_drive_published:
#             self.ego_drive_published = True
#         self.ego_requested_speed = twist_msg.linear.x
#         self.ego_steer = 0.3 if twist_msg.angular.z > 0.0 else -0.3 if twist_msg.angular.z < 0.0 else 0.0

#     def drive_timer_callback(self):
#         """Step simulation"""
#         if self.ego_drive_published and not self.has_opp:
#             self.obs = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed]]))
#         elif self.ego_drive_published and self.has_opp and self.opp_drive_published:
#             self.obs = self.env.step(np.array([
#                 [self.ego_steer, self.ego_requested_speed],
#                 [self.opp_steer, self.opp_requested_speed]
#             ]))
#         self._update_sim_state()

#     def timer_callback(self):
#         """Publish sensor data and transforms"""
#         ts = self.get_clock().now().to_msg()
        
#         # Publish ego scan
#         scan = LaserScan()
#         scan.header.stamp = ts
#         scan.header.frame_id = self.ego_namespace + '/laser'
#         scan.angle_min = self.angle_min
#         scan.angle_max = self.angle_max
#         scan.angle_increment = self.angle_inc
#         scan.range_min = 0.
#         scan.range_max = 30.
#         scan.ranges = self.ego_scan
#         self.ego_scan_pub.publish(scan)
        
#         # Publish opponent scan
#         if self.has_opp:
#             opp_scan = LaserScan()
#             opp_scan.header.stamp = ts
#             opp_scan.header.frame_id = self.opp_namespace + '/laser'
#             opp_scan.angle_min = self.angle_min
#             opp_scan.angle_max = self.angle_max
#             opp_scan.angle_increment = self.angle_inc
#             opp_scan.range_min = 0.
#             opp_scan.range_max = 30.
#             opp_scan.ranges = self.opp_scan
#             self.opp_scan_pub.publish(opp_scan)
        
#         # Publish odometry and transforms
#         self._publish_odom(ts)
#         self._publish_transforms(ts)
#         self._publish_laser_transforms(ts)
#         self._publish_wheel_transforms(ts)

#     def _update_sim_state(self):
#         """Update stored states from simulator"""
#         self.ego_scan = list(self.obs['scans'][0])
        
#         if self.has_opp:
#             self.opp_scan = list(self.obs['scans'][1])
#             self.opp_pose[0] = self.obs['poses_x'][1]
#             self.opp_pose[1] = self.obs['poses_y'][1]
#             self.opp_pose[2] = self.obs['poses_theta'][1]
#             self.opp_speed[0] = self.obs['linear_vels_x'][1]
#             self.opp_speed[1] = self.obs['linear_vels_y'][1]
#             self.opp_speed[2] = self.obs['ang_vels_z'][1]
        
#         self.ego_pose[0] = self.obs['poses_x'][0]
#         self.ego_pose[1] = self.obs['poses_y'][0]
#         self.ego_pose[2] = self.obs['poses_theta'][0]
#         self.ego_speed[0] = self.obs['linear_vels_x'][0]
#         self.ego_speed[1] = self.obs['linear_vels_y'][0]
#         self.ego_speed[2] = self.obs['ang_vels_z'][0]

#     def _publish_odom(self, ts):
#         """Publish odometry messages"""
#         # Ego odom
#         ego_odom = Odometry()
#         ego_odom.header.stamp = ts
#         ego_odom.header.frame_id = 'map'
#         ego_odom.child_frame_id = self.ego_namespace + '/base_link'
#         ego_odom.pose.pose.position.x = self.ego_pose[0]
#         ego_odom.pose.pose.position.y = self.ego_pose[1]
        
#         ego_quat = self.euler_to_quaternion(0., 0., self.ego_pose[2])
#         ego_odom.pose.pose.orientation.x = ego_quat[0]
#         ego_odom.pose.pose.orientation.y = ego_quat[1]
#         ego_odom.pose.pose.orientation.z = ego_quat[2]
#         ego_odom.pose.pose.orientation.w = ego_quat[3]
        
#         ego_odom.twist.twist.linear.x = self.ego_speed[0]
#         ego_odom.twist.twist.linear.y = self.ego_speed[1]
#         ego_odom.twist.twist.angular.z = self.ego_speed[2]
#         self.ego_odom_pub.publish(ego_odom)
        
#         # Opponent odom
#         if self.has_opp:
#             opp_odom = Odometry()
#             opp_odom.header.stamp = ts
#             opp_odom.header.frame_id = 'map'
#             opp_odom.child_frame_id = self.opp_namespace + '/base_link'
#             opp_odom.pose.pose.position.x = self.opp_pose[0]
#             opp_odom.pose.pose.position.y = self.opp_pose[1]
            
#             opp_quat = self.euler_to_quaternion(0., 0., self.opp_pose[2])
#             opp_odom.pose.pose.orientation.x = opp_quat[0]
#             opp_odom.pose.pose.orientation.y = opp_quat[1]
#             opp_odom.pose.pose.orientation.z = opp_quat[2]
#             opp_odom.pose.pose.orientation.w = opp_quat[3]
            
#             opp_odom.twist.twist.linear.x = self.opp_speed[0]
#             opp_odom.twist.twist.linear.y = self.opp_speed[1]
#             opp_odom.twist.twist.angular.z = self.opp_speed[2]
#             self.opp_odom_pub.publish(opp_odom)
#             self.opp_ego_odom_pub.publish(ego_odom)
#             self.ego_opp_odom_pub.publish(opp_odom)

#     def _publish_transforms(self, ts):
#         """Publish TF transforms"""
#         # Ego transform
#         ego_t = Transform()
#         ego_t.translation.x = self.ego_pose[0]
#         ego_t.translation.y = self.ego_pose[1]
#         ego_t.translation.z = 0.0
        
#         ego_quat = self.euler_to_quaternion(0.0, 0.0, self.ego_pose[2])
#         ego_t.rotation.x = ego_quat[0]
#         ego_t.rotation.y = ego_quat[1]
#         ego_t.rotation.z = ego_quat[2]
#         ego_t.rotation.w = ego_quat[3]
        
#         ego_ts = TransformStamped()
#         ego_ts.transform = ego_t
#         ego_ts.header.stamp = ts
#         ego_ts.header.frame_id = 'map'
#         ego_ts.child_frame_id = self.ego_namespace + '/base_link'
#         self.br.sendTransform(ego_ts)
        
#         # Opponent transform
#         if self.has_opp:
#             opp_t = Transform()
#             opp_t.translation.x = self.opp_pose[0]
#             opp_t.translation.y = self.opp_pose[1]
#             opp_t.translation.z = 0.0
            
#             opp_quat = self.euler_to_quaternion(0.0, 0.0, self.opp_pose[2])
#             opp_t.rotation.x = opp_quat[0]
#             opp_t.rotation.y = opp_quat[1]
#             opp_t.rotation.z = opp_quat[2]
#             opp_t.rotation.w = opp_quat[3]
            
#             opp_ts = TransformStamped()
#             opp_ts.transform = opp_t
#             opp_ts.header.stamp = ts
#             opp_ts.header.frame_id = 'map'
#             opp_ts.child_frame_id = self.opp_namespace + '/base_link'
#             self.br.sendTransform(opp_ts)

#     def _publish_wheel_transforms(self, ts):
#         """Publish TF for front wheels steering"""
#         # Ego left and right front wheel hinges
#         ego_wheel_ts = TransformStamped()
#         ego_wheel_quat = self.euler_to_quaternion(0., 0., self.ego_steer)
#         ego_wheel_ts.transform.rotation.x = ego_wheel_quat[0]
#         ego_wheel_ts.transform.rotation.y = ego_wheel_quat[1]
#         ego_wheel_ts.transform.rotation.z = ego_wheel_quat[2]
#         ego_wheel_ts.transform.rotation.w = ego_wheel_quat[3]
#         ego_wheel_ts.header.stamp = ts
#         ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
#         ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
#         self.br.sendTransform(ego_wheel_ts)

#         ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
#         ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
#         self.br.sendTransform(ego_wheel_ts)

#         # Opponent wheels
#         if self.has_opp:
#             opp_wheel_ts = TransformStamped()
#             opp_wheel_quat = self.euler_to_quaternion(0., 0., self.opp_steer)
#             opp_wheel_ts.transform.rotation.x = opp_wheel_quat[0]
#             opp_wheel_ts.transform.rotation.y = opp_wheel_quat[1]
#             opp_wheel_ts.transform.rotation.z = opp_wheel_quat[2]
#             opp_wheel_ts.transform.rotation.w = opp_wheel_quat[3]
#             opp_wheel_ts.header.stamp = ts
#             opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_left_hinge'
#             opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
#             self.br.sendTransform(opp_wheel_ts)

#             opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
#             opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
#             self.br.sendTransform(opp_wheel_ts)

#     def _publish_laser_transforms(self, ts):
#         """Publish TF transform for LIDAR scan frame"""
#         ego_scan_ts = TransformStamped()
#         ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
#         ego_scan_ts.transform.rotation.w = 1.
#         ego_scan_ts.header.stamp = ts
#         ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
#         ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
#         self.br.sendTransform(ego_scan_ts)
        
#         # Opponent LIDAR frame
#         if self.has_opp:
#             opp_scan_ts = TransformStamped()
#             opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
#             opp_scan_ts.transform.rotation.w = 1.
#             opp_scan_ts.header.stamp = ts
#             opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
#             opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
#             self.br.sendTransform(opp_scan_ts)


# def main(args=None):
#     """Entry point for running the ROS2 gym bridge node"""
#     rclpy.init(args=args)
#     gym_bridge = GymBridge()
#     rclpy.spin(gym_bridge)
#     gym_bridge.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
        



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

        if self.kb_teleop:
            self.create_subscription(Twist, '/cmd_vel', self.teleop_cb, 10)

        # ===== Timers =====
        self.create_timer(self.dt, self.step_sim)
        self.get_logger().info(f"✅ GymBridge started with {self.num_agents} agents(s)")

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
        self.obs, self.reward, self.done, self.info = self.env.step(self.actions)
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
    