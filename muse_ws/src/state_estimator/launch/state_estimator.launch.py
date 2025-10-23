#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    use_contact_data_packaging_arg = DeclareLaunchArgument(
        'use_contact_data_packaging',
        default_value='false',
        description='Use ContactDataPackagingPlugin instead of ContactPlugin'
    )
    
    # Load configuration files
    attitude_config = os.path.join(pkg_dir, 'config', 'attitude_plugin.yaml')
    
    # Conditionally select contact plugin configuration
    contact_plugin_config = os.path.join(
        pkg_dir, 'config', 'contact_plugin.yaml'
    )
    contact_data_packaging_config = os.path.join(
        pkg_dir, 'config', 'contact_data_packaging_plugin.yaml'
    )
    leg_odometry_config = os.path.join(
        pkg_dir, 'config', 'leg_odometry.yaml'
    )
    sensor_fusion_config = os.path.join(
        pkg_dir, 'config', 'sensor_fusion.yaml'
    )
    
    # Function to launch the node with conditional config
    def launch_node(context, *args, **kwargs):
        use_packaging = LaunchConfiguration(
            'use_contact_data_packaging'
        ).perform(context)
        
        # Select the appropriate contact plugin config
        if use_packaging.lower() == 'true':
            contact_config = contact_data_packaging_config
        else:
            contact_config = contact_plugin_config
        
        # Create the node with selected configuration
        node = Node(
            package='state_estimator',
            executable='state_estimator_node',
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
        return [node]
    
    return LaunchDescription([
        pluginlist_yaml_arg,
        timeoutlist_yaml_arg,
        use_contact_data_packaging_arg,
        OpaqueFunction(function=launch_node)
    ])
