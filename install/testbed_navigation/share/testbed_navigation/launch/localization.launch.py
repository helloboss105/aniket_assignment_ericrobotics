#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    testbed_navigation_dir = get_package_share_directory('testbed_navigation')
    testbed_bringup_dir = get_package_share_directory('testbed_bringup')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([testbed_navigation_dir, 'config', 'amcl_params.yaml']),
        description='Full path to the ROS2 parameters file to use for AMCL'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([testbed_bringup_dir, 'maps', 'testbed_world.yaml']),
        description='Full path to map yaml file to load'
    )
    
    use_map_server_arg = DeclareLaunchArgument(
        'use_map_server',
        default_value='true',
        description='Whether to start the map server'
    )
    
    # Include map server launch if requested
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([testbed_navigation_dir, 'launch', 'map_loader.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('use_map_server')),
        launch_arguments={'map': LaunchConfiguration('map'),
                         'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )
    
    # AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Lifecycle manager for localization
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'node_names': ['amcl'],
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        params_file_arg,
        map_arg,
        use_map_server_arg,
        map_server_launch,
        amcl_node,
        lifecycle_manager_node
    ])