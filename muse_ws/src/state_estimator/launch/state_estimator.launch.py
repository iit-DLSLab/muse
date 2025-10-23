#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('state_estimator')
    
    # Declare launch arguments with default values
    pluginlist_yaml_arg = DeclareLaunchArgument(
        'pluginlist_yaml',
        default_value=os.path.join(pkg_dir, 'launch', 'pluginlist.yaml'),
        description='Path to plugin list YAML file'
    )
    
    timeoutlist_yaml_arg = DeclareLaunchArgument(
        'timeoutlist_yaml',
        default_value=os.path.join(pkg_dir, 'launch', 'timeout.yaml'),
        description='Path to timeout list YAML file'
    )
    
    # Load configuration files
    attitude_config = os.path.join(pkg_dir, 'config', 'attitude_plugin.yaml')
    contact_config = os.path.join(pkg_dir, 'config', 'contact_plugin.yaml')
    leg_odometry_config = os.path.join(pkg_dir, 'config', 'leg_odometry.yaml')
    sensor_fusion_config = os.path.join(pkg_dir, 'config', 'sensor_fusion.yaml')
    
    # Create the node
    state_estimator_node = Node(
        package='state_estimator',
        executable='state_estimator_node',  # This should match the executable name in CMakeLists.txt
        name='state_estimator',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            LaunchConfiguration('pluginlist_yaml'),
            LaunchConfiguration('timeoutlist_yaml'),
            attitude_config,
            contact_config,
            leg_odometry_config,
            sensor_fusion_config
        ],
        respawn=False
    )
    
    return LaunchDescription([
        pluginlist_yaml_arg,
        timeoutlist_yaml_arg,
        state_estimator_node
    ])
