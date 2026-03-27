from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("state_estimator"))

    # The config/launch YAML files are in ROS1 plugin-owned format and are read
    # directly by the node via yaml-cpp -- they cannot be fed as ROS2 --params-file
    # because they lack the required "node_name/ros__parameters" wrapping, use
    # ROS1-only $(find ...) substitution, and contain sequences-of-maps that the
    # ROS2 params parser does not support.
    #
    # Instead we expose the two directory paths as ordinary ROS2 string parameters
    # so the (ported) node can locate and load them at runtime.
    node_params = [
        {
            "config_dir": str(package_share / "config"),
            "launch_dir": str(package_share / "launch"),
        }
    ]

    return LaunchDescription(
        [
            Node(
                package="state_estimator",
                executable="state_estimator_node",
                name="state_estimator",
                output="screen",
                parameters=node_params,
            )
        ]
    )
