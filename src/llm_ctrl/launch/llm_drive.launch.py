from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='llm_ctrl',
            executable='llm_drive',
            name='llm_drive_node',
            output='screen'
        ),
    ])

