#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    testbed_bringup_dir = get_package_share_directory('testbed_bringup')
    
    # Declare launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([testbed_bringup_dir, 'maps', 'testbed_world.yaml']),
        description='Full path to map yaml file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'map'
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Lifecycle manager for map server - delayed start
    lifecycle_manager_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                parameters=[{
                    'node_names': ['map_server'],
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': True,
                    'bond_timeout': 4.0
                }]
            )
        ]
    )
    
    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        map_server_node,
        lifecycle_manager_node
    ])
