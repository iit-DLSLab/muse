from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    state_estimator_launch = (
        get_package_share_directory("state_estimator") + "/launch/state_estimator.launch.py"
    )

    point_lio_bridge_launch = (
        get_package_share_directory("muse_point_lio") + "/launch/point_lio_bridge.launch.py"
    )

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_point_lio", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("point_lio_package", default_value="point_lio"),
        DeclareLaunchArgument("point_lio_launch_file", default_value="mapping_go2_muse.launch.py"),
        DeclareLaunchArgument("point_lio_odom_topic", default_value="/point_lio/odometry"),
        DeclareLaunchArgument("lidar_odometry_topic", default_value="/lidar_odometry"),
        DeclareLaunchArgument("bridge_frame_id", default_value=""),
        DeclareLaunchArgument("bridge_child_frame_id", default_value=""),
    ]

    se = IncludeLaunchDescription(PythonLaunchDescriptionSource(state_estimator_launch))

    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(point_lio_bridge_launch),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "start_point_lio": LaunchConfiguration("start_point_lio"),
            "rviz": LaunchConfiguration("rviz"),
            "point_lio_package": LaunchConfiguration("point_lio_package"),
            "point_lio_launch_file": LaunchConfiguration("point_lio_launch_file"),
            "point_lio_odom_topic": LaunchConfiguration("point_lio_odom_topic"),
            "lidar_odometry_topic": LaunchConfiguration("lidar_odometry_topic"),
            "bridge_frame_id": LaunchConfiguration("bridge_frame_id"),
            "bridge_child_frame_id": LaunchConfiguration("bridge_child_frame_id"),
        }.items(),
    )

    return LaunchDescription(args + [se, point_lio])
