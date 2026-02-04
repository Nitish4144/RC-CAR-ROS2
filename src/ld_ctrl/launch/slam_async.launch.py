import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch SLAM Toolbox (async) with YDLiDAR
    ROS 2 Jazzy compatible
    """

    # Use sim time (optional, default false)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Path to SLAM config
    slam_config = PathJoinSubstitution([
        FindPackageShare('ld_ctrl'),
        'config',
        'slam_toolbox_async.yaml'
    ])

    # --------------------------------------------
    # 1. Launch LiDAR + base setup (ld_ctrl)
    # --------------------------------------------
    ld_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ld_ctrl'),
                'launch',
                'ld_ctrl.launch.py'
            ])
        )
    )

    # --------------------------------------------
    # 2. Static TF: base_link → laser_frame
    # --------------------------------------------
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        output='log',
        arguments=[
            '0.0', '0.0', '0.15',   # x y z
            '0.0', '0.0', '0.0',    # roll pitch yaw
            'base_link',
            'laser_frame'
        ]
    )

    # --------------------------------------------
    # 3. SLAM Toolbox (Async)
    # --------------------------------------------
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan', '/scan'),
        ]
    )

    # --------------------------------------------
    # Launch description
    # --------------------------------------------
    return LaunchDescription([
        ld_ctrl_launch,
        static_tf,
        slam_node
    ])
