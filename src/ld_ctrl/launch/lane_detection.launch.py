from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch lane detection and control nodes
    Outputs Ackermann steering commands
    """

    lane_detector = Node(
        package='ld_ctrl',
        executable='lane_detector',
        name='lane_detector',
        output='screen',
        remappings=[
            # Change this ONLY if your camera topic differs
            ('image_raw', '/camera/image_raw')
        ]
    )

    lane_controller = Node(
        package='ld_ctrl',
        executable='lane_controller',
        name='lane_controller',
        output='screen',
        remappings=[
            # Change this ONLY if your vehicle expects a different topic
            ('drive', '/drive_ackermann')
        ]
    )

    return LaunchDescription([
        lane_detector,
        lane_controller
    ])
