from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch YDLiDAR driver using official launch file
    (ROS 2 Jazzy compatible)
    """

    # Path to YDLiDAR driver package
    ydlidar_driver_share = FindPackageShare('ydlidar_ros2_driver')

    # Path to YOUR YDLiDAR config
    config_file = PathJoinSubstitution([
        FindPackageShare('ld_ctrl'),
        'config',
        'ydlidar.yaml'
    ])

    # Include official YDLiDAR launch file
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ydlidar_driver_share,
                'launch',
                'ydlidar_launch_view.py'
            ])
        ),
        launch_arguments={
            'params_file': config_file
        }.items()
    )

    return LaunchDescription([
        ydlidar_launch
    ])
