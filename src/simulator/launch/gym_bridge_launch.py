# # MIT License

# # Copyright (c) 2024 S7our Squad
# # Permission is hereby granted, free of charge, to any person obtaining a copy
# # of this software and associated documentation files (the "Software"), to deal
# # in the Software without restriction, including without limitation the rights
# # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# # copies of the Software, and to permit persons to whom the Software is
# # furnished to do so, subject to the following conditions:

# # The above copyright notice and this permission notice shall be included in all
# # copies or substantial portions of the Software.

# # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# # SOFTWARE.

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import Command
# from ament_index_python.packages import get_package_share_directory
# import os
# import yaml

# def generate_launch_description():
#     ld = LaunchDescription()
#     config = os.path.join(
#         get_package_share_directory('simulator'),
#         'config',
#         'sim.yaml'
#         )
#     config_dict = yaml.safe_load(open(config, 'r'))
#     has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
#     teleop = config_dict['bridge']['ros__parameters']['kb_teleop']

#     bridge_node = Node(
#         package='simulator',
#         executable='gym_bridge',
#         name='bridge',
#         parameters=[config]
#     )
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz',
#         arguments=['-d', os.path.join(get_package_share_directory('simulator'), 'launch', 'gym_bridge.rviz')]
#     )
#     map_server_node = Node(
#         package='nav2_map_server',
#         executable='map_server',
#         parameters=[{'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
#                     {'topic': 'map'},
#                     {'frame_id': 'map'},
#                     {'output': 'screen'},
#                     {'use_sim_time': True}]
#     )
#     nav_lifecycle_node = Node(
#         package='nav2_lifecycle_manager',
#         executable='lifecycle_manager',
#         name='lifecycle_manager_localization',
#         output='screen',
#         parameters=[{'use_sim_time': True},
#                     {'autostart': True},
#                     {'node_names': ['map_server']}]
#     )
#     ego_robot_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='ego_robot_state_publisher',
#         parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('simulator'), 'launch', 'ego_racecar.xacro')])}],
#         remappings=[('/robot_description', 'ego_robot_description')]
#     )
#     opp_robot_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='opp_robot_state_publisher',
#         parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('simulator'), 'launch', 'opp_racecar.xacro')])}],
#         remappings=[('/robot_description', 'opp_robot_description')]
#     )

#     # finalize
#     ld.add_action(rviz_node)
#     ld.add_action(bridge_node)
#     ld.add_action(nav_lifecycle_node)
#     ld.add_action(map_server_node)
#     ld.add_action(ego_robot_publisher)
#     if has_opp:
#         ld.add_action(opp_robot_publisher)

#     return ld

#!/usr/bin/env python3
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    # pkg_share = FindPackageShare('simulator').find('simulator')
    pkg_share = get_package_share_directory('simulator')

    
    # Paths to files
    ego_xacro_file = os.path.join(pkg_share, 'launch', 'ego_racecar.xacro')
    rviz_config_file = os.path.join(pkg_share, 'launch', 'gym_bridge.rviz')
    params_file = os.path.join(pkg_share, 'config', 'sim.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'levine')

    config = os.path.join(
         get_package_share_directory('simulator'),
         'config',
         'sim.yaml'
         )
    config_dict = yaml.safe_load(open(config, 'r'))
    has_opp = config_dict['gym_bridge']['ros__parameters']['num_agent'] > 1
    teleop = config_dict['gym_bridge']['ros__parameters']['kb_teleop']


    # Declare launch arguments
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=default_map,
        description='Path to the map file (without extension)'
    )
    
    num_agent_arg = DeclareLaunchArgument(
        'num_agent',
        default_value='1',
        description='Number of agents (1 or 2)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    # Get launch configurations
    map_path = LaunchConfiguration('map_path')
    num_agent = LaunchConfiguration('num_agent')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Process xacro to generate URDF for ego vehicle
    # Note: ego_robot_description is only needed if used later, but Command() is sufficient
    # for direct use in robot_state_publisher.
    
    # Gym Bridge Node
    gym_bridge_node = Node(
        package='simulator',
        executable='gym_bridge',
        name='gym_bridge',
        output='screen',
        parameters=[
            params_file,
            {
                'map_path': map_path,
                'num_agent': num_agent,
            }
        ],
        emulate_tty=True,
    )

    # Map Server Node to publish the OccupancyGrid to /map topic
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                # FIX: Use a list to concatenate LaunchConfiguration and string literal
                # 'yaml_filename': [map_path, '.yaml'],

                'yaml_filename': PythonExpression(["'", map_path, ".yaml'"]),

                'topic_name': 'map',
                'frame_id': 'map',
                'use_sim_time': False
            }
        ]
    )

    # ADDED: Lifecycle Manager to activate the map_server node (Required for Nav2 nodes)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server']} # This manages the node named 'map_server'
        ]
    )
    
    
    # Robot State Publisher for ego vehicle
    robot_state_publisher_ego= Node(
         package='robot_state_publisher',
         executable='robot_state_publisher',
         name='ego_robot_state_publisher',
         parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('simulator'), 'launch', 'ego_racecar.xacro')])}],
         remappings=[('/robot_description', 'ego_robot_description')]
    )
    
    # Joint State Publisher for ego (publishes static transforms)
    joint_state_publisher_ego = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='ego_racecar',
        parameters=[{
            'use_sim_time': False,
        }],
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz),
    )
    
    return LaunchDescription([
        # Arguments
        map_path_arg,
        num_agent_arg,
        use_rviz_arg,
        
        # Nodes
        gym_bridge_node,
        map_server_node,
        lifecycle_manager_node, # <-- CRITICAL: Activates the map_server
        robot_state_publisher_ego,
        # robot_state_publisher_opp,  # Uncomment for 2 agents
        joint_state_publisher_ego,
        rviz_node,
    ])