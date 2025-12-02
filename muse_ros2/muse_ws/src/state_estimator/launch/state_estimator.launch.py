#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('state_estimator')
    
    # Declare launch arguments
    pluginlist_yaml_arg = DeclareLaunchArgument(
        'pluginlist_yaml',
        default_value=PathJoinSubstitution([
            pkg_share, 'launch', 'pluginlist.yaml'
        ]),
        description='Path to pluginlist YAML file'
    )
    
    timeoutlist_yaml_arg = DeclareLaunchArgument(
        'timeoutlist_yaml',
        default_value=PathJoinSubstitution([
            pkg_share, 'launch', 'timeout.yaml'
        ]),
        description='Path to timeout YAML file'
    )
    
    # Configure parameter files
    attitude_plugin_config = PathJoinSubstitution([
        pkg_share, 'config', 'attitude_plugin.yaml'
    ])
    
    contact_plugin_config = PathJoinSubstitution([
        pkg_share, 'config', 'contact_plugin.yaml'
    ])
    
    leg_odometry_config = PathJoinSubstitution([
        pkg_share, 'config', 'leg_odometry.yaml'
    ])
    
    sensor_fusion_config = PathJoinSubstitution([
        pkg_share, 'config', 'sensor_fusion.yaml'
    ])
    
    # State estimator node
    state_estimator_node = Node(
        package='state_estimator',
        executable='state_estimator_node',
        name='state_estimator',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            LaunchConfiguration('pluginlist_yaml'),
            LaunchConfiguration('timeoutlist_yaml'),
            attitude_plugin_config,
            contact_plugin_config,
            leg_odometry_config,
            sensor_fusion_config,
        ],
        respawn=False,
    )
    
    return LaunchDescription([
        pluginlist_yaml_arg,
        timeoutlist_yaml_arg,
        state_estimator_node,
    ])
