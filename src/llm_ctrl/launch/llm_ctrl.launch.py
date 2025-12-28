from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='llm_ctrl',
            executable='controller',
            name='controller_node',
            output='screen'
        ),
        Node(
            package='llm_ctrl',
            executable='llm_drive',
            name='llm_drive_node',
            output='screen'
        ),
        Node(
            package='manual_ctrl',
            executable='motor_signals',
            name='motor_signals_node',
            output='screen'
        ),
    ])
