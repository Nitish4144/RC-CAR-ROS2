from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manual_ctrl',
            executable='joy_raw',
            name='joy_raw_node',
            output='screen'
        ),
        Node(
            package='manual_ctrl',
            executable='joy_drive',
            name='joy_drive_node',
            output='screen'
        ),
        Node(
            package='manual_ctrl',
            executable='motor_signals',
            name='motor_signals_node',
            output='screen'
        ),
    ])
