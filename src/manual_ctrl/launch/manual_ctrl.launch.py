from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """
    Main launch file for complete RC-CAR-ROS2 system with LiDAR.
    
    Includes:
    - Manual Control System (your existing nodes)
    - LiDAR Driver
    - Autonomous Navigation Nodes
    - Safety Monitor
    """
    
    ld = LaunchDescription()
    
    # ============================================
    # MANUAL CONTROL SYSTEM (YOUR EXISTING NODES)
    # ============================================
    joy_raw_node = Node(
        package='manual_ctrl',
        executable='joy_raw',
        name='joy_raw_node',
        output='screen'
    )
    ld.add_action(joy_raw_node)
    
    joy_drive_node = Node(
        package='manual_ctrl',
        executable='joy_drive',
        name='joy_drive_node',
        output='screen'
    )
    ld.add_action(joy_drive_node)
    
    motor_signals_node = Node(
        package='manual_ctrl',
        executable='motor_signals',
        name='motor_signals_node',
        output='screen'
    )
    ld.add_action(motor_signals_node)
    
    # ============================================
    # LIDAR DRIVER
    # ============================================
    ld_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ld_ctrl'),
                'launch',
                'ld_ctrl.launch.py'
            ])
        )
    )
    ld.add_action(ld_ctrl_launch)
    
    # ============================================
    # LIDAR FEEDBACK FOR MANUAL MODE
    # ============================================
    ld_ctrl_manual_node = Node(
        package='manual_ctrl',
        executable='ld_ctrl_manual',
        output='screen',
        parameters=[{
            'warning_threshold': 1.0,
            'danger_threshold': 0.5,
            'log_interval': 10,
        }]
    )
    ld.add_action(ld_ctrl_manual_node)
    return ld
