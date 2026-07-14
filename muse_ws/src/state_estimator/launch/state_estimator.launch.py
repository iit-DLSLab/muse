from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("state_estimator")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    parameter_files = [
        PathJoinSubstitution([package_share, "launch", "pluginlist.yaml"]),
        PathJoinSubstitution([package_share, "config", "attitude_plugin.yaml"]),
        PathJoinSubstitution([package_share, "config", "contact_plugin.yaml"]),
        PathJoinSubstitution([package_share, "config", "leg_odometry.yaml"]),
        PathJoinSubstitution([package_share, "config", "sensor_fusion.yaml"]),
        PathJoinSubstitution([package_share, "config", "tf_publisher.yaml"]),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "state_estimator.rviz"]
                ),
            ),
            Node(
                package="state_estimator",
                executable="state_estimator_node",
                name="state_estimator",
                output="screen",
                parameters=parameter_files + [{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(launch_rviz),
                output="screen",
            ),
        ]
    )
