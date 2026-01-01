from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """
    Launch LiDAR driver with X2 configuration.
    This is included by other launch files.
    """
    
    # Get the LiDAR driver package location
    ydlidar_driver_share = FindPackageShare('ydlidar_ros2_driver')
    
    # Path to X2 config file
    config_file = PathJoinSubstitution([
        FindPackageShare('ld_ctrl'),
        'config',
        'ydlidar.yaml'
    ])
    
    # Include the LiDAR driver launch file with our config
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ydlidar_driver_share,
                'launch',
                'ydlidar_launch_view.py'  # Includes RViz
            ])
        ),
        launch_arguments={
            'params_file': config_file
        }.items()
    )
    
    ld = LaunchDescription([ydlidar_launch])
    return ld
