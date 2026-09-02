"""Real AOS mapping stage: sensors/odometry + SLAM Toolbox + RViz only."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("m20_nav2_system"))
    slam_params = str(share / "config" / "slam_toolbox.yaml")
    rviz_config = str(share / "rviz" / "nav2_sandbox.rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    locomotion_launch = PathJoinSubstitution([
        FindPackageShare("m20_nav2_locomotion"), "launch", "sdk_locomotion.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("start_sdk", default_value="false",
                              description="Start the official SDK command adapter"),
        Node(
            package="slam_toolbox", executable="async_slam_toolbox_node",
            name="slam_toolbox", output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time,
                                      "mode": "mapping"}],
            remappings=[("scan", LaunchConfiguration("scan_topic")),
                        ("odom", LaunchConfiguration("odom_topic"))],
        ),
        # During mapping, the SDK is only the manual motion backend. Nav2 is
        # intentionally absent, so this node cannot receive autonomous goals.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(locomotion_launch),
            launch_arguments={
                "start_sdk": LaunchConfiguration("start_sdk"),
                "require_backend_ready": "False",
                "input_topic": "/cmd_vel",
                "sdk_cmd_vel_topic": "/m20/locomotion/cmd_vel_sdk",
                "execution_profile": "scan_native",
            }.items(),
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
