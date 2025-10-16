from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Manual_ctrl',
            executable='motor_signals',
            name='motor_signals_node',
            output='screen'
        ),
    ])
