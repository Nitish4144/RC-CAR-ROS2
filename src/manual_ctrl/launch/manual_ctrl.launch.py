from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """
    Main launch file for RC-CAR-ROS2 system.

    Includes:
    - Manual Control System
    - SLAM Mapping
    - Safety Monitor
    """

    ld = LaunchDescription()

    # ============================================
    # MANUAL CONTROL SYSTEM
    # ============================================
    joy_raw_node = Node(
        package='manual_ctrl',
        executable='joy_raw',
        name='joy_raw_node',
        output='screen'
    )

    joy_drive_node = Node(
        package='manual_ctrl',
        executable='joy_drive',
        name='joy_drive_node',
        output='screen'
    )

    motor_signals_node = Node(
        package='manual_ctrl',
        executable='motor_signals',
        name='motor_signals_node',
        output='screen'
    )

    ld.add_action(joy_raw_node)
    ld.add_action(joy_drive_node)
    ld.add_action(motor_signals_node)

    # ============================================
    # SLAM MAPPING
    # ============================================
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ld_ctrl'),
                'launch',
                'slam_async.launch.py'
            ])
        )
    )

    #ld.add_action(slam_launch)

    # ============================================
    # SAFETY MONITORING NODE
    # ============================================
    safety_node = Node(
        package='safety_node',
        executable='safety_node',
        name='safety_node',
        output='screen',
        parameters=[{
            'collision_threshold': 0.3,
            'warning_threshold': 0.5,
        }]
    )

    #ld.add_action(safety_node)

    return ld

