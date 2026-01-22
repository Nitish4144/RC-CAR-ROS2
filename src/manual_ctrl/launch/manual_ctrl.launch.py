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
    # LIDAR + SLAM MAPPING
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
    ld.add_action(slam_launch)

    
    
    # ============================================
    # LIDAR DRIVER
    # ============================================
    # ld_ctrl_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('ld_ctrl'),
    #             'launch',
    #             'ld_ctrl.launch.py'
    #         ])
    #     )
    # )
    # ld.add_action(ld_ctrl_launch)
    
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
    
    # ============================================
    # SAFETY MONITORING NODE
    # ============================================
    safety_node = Node(
        package='safety_node',
        executable='safety_node',
        output='screen',
        parameters=[{
            'collision_threshold': 0.3,
            'warning_threshold': 0.5,
        }]
    )
    ld.add_action(safety_node)
    
    # ============================================
    # AUTONOMOUS NAVIGATION NODES
    # ============================================
    
    # Gap Following
    gap_follow_node = Node(
        package='gap_follow',
        executable='gap_follow_node',
        output='screen',
        parameters=[{
            'linear_velocity': 0.5,
            'gap_threshold': 0.5,
            'min_gap_width': 10,
        }]
    )
    ld.add_action(gap_follow_node)
    
    # Wall Following
    wall_follow_node = Node(
        package='wall_follow',
        executable='wall_follow_node',
        output='screen',
        parameters=[{
            'target_distance': 0.5,
            'linear_velocity': 0.3,
            'kp': 1.0,
            'wall_side': 'left',
        }]
    )
    ld.add_action(wall_follow_node)
    
    # LLM Control
   # llm_ctrl_node = Node(
   #     package='llm_ctrl',
   #     executable='llm_ctrl_node',
   #     output='screen'
   # )
   # ld.add_action(llm_ctrl_node)
    
    return ld
