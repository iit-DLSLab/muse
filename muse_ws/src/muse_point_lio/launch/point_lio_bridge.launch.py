from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _create_odom_bridge_process(context):
    cmd = [
        FindExecutable(name="python3").perform(context),
        "-m",
        "muse_point_lio.odom_bridge",
        "--ros-args",
        "-r",
        "__node:=point_lio_odom_bridge",
        "-p",
        f"input_topic:={LaunchConfiguration('point_lio_odom_topic').perform(context)}",
        "-p",
        f"output_topic:={LaunchConfiguration('lidar_odometry_topic').perform(context)}",
        "-p",
        "use_input_stamp:=true",
    ]

    frame_id = LaunchConfiguration("bridge_frame_id").perform(context).strip()
    child_frame_id = LaunchConfiguration("bridge_child_frame_id").perform(context).strip()

    if frame_id:
        cmd.extend(["-p", f"frame_id:={frame_id}"])

    if child_frame_id:
        cmd.extend(["-p", f"child_frame_id:={child_frame_id}"])

    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
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

    point_lio_launch = IncludeLaunchDescription(
        condition=IfCondition(LaunchConfiguration("start_point_lio")),
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("point_lio_package")),
                    "launch",
                    LaunchConfiguration("point_lio_launch_file"),
                ]
            )
        ),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    odom_bridge_node = OpaqueFunction(function=_create_odom_bridge_process)

    return LaunchDescription(args + [point_lio_launch, odom_bridge_node])
