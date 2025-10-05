#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    testbed_navigation_dir = get_package_share_directory('testbed_navigation')
    
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
        default_value=PathJoinSubstitution([testbed_navigation_dir, 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Whether to use composed bringup'
    )
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Whether to respawn if a node crashes'
    )

    # Create the launch description and populate
    bringup_cmd_group = GroupAction([
        Node(
            condition=IfCondition(LaunchConfiguration('use_composition')),
            package='rclcpp_components',
            name='nav2_container',
            executable='component_container_isolated',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')],
            output='screen'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')]),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')]),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')]),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('cmd_vel', 'cmd_vel_nav'),
                        ('cmd_vel_smoothed', 'cmd_vel')]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'node_names': ['controller_server',
                                        'planner_server',
                                        'smoother_server',
                                        'behavior_server',
                                        'bt_navigator',
                                        'waypoint_follower',
                                        'velocity_smoother']},
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'autostart': LaunchConfiguration('autostart')}]),
    ])

    # Set environment variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    return LaunchDescription([
        # Set env var to print messages immediately to stdout
        stdout_linebuf_envvar,
        
        use_sim_time_arg,
        autostart_arg,
        params_file_arg,
        use_composition_arg,
        use_respawn_arg,
        
        bringup_cmd_group,
    ])
