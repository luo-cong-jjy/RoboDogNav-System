"""Start the M20 Nav2 stack against real AOS sensors and the official SDK."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = Path(get_package_share_directory("m20_nav2_system"))
    description_share = Path(
        get_package_share_directory("m20_nav2_description")
    )
    default_params = str(nav_share / "config" / "nav2_params.yaml")
    default_rviz = str(nav_share / "rviz" / "nav2_sandbox.rviz")
    default_description = str(
        description_share / "urdf" / "m20_official.urdf"
    )
    navigation_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "bringup_launch.py",
    ])
    locomotion_launch = PathJoinSubstitution([
        FindPackageShare("m20_nav2_locomotion"),
        "launch",
        "sdk_locomotion.launch.py",
    ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    start_sdk = LaunchConfiguration("start_sdk")

    return LaunchDescription([
        DeclareLaunchArgument("map", description="Verified real-world map YAML"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument(
            "scan_topic", default_value="/scan",
            description="Real AOS LaserScan topic",
        ),
        DeclareLaunchArgument(
            "odom_topic", default_value="/odom",
            description="Real base odometry topic from AOS, LIO, or another estimator",
        ),
        DeclareLaunchArgument(
            "start_sdk", default_value="false",
            description="Enable rl_deploy_cmdvel only after safety checks",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="m20_robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Path(default_description).read_text(
                    encoding="utf-8"
                ),
                "use_sim_time": use_sim_time,
            }],
        ),
        GroupAction([
            SetRemap(src="/scan", dst=LaunchConfiguration("scan_topic")),
            SetRemap(src="/odom", dst=LaunchConfiguration("odom_topic")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": "True",
                    "use_composition": "False",
                }.items(),
            ),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(locomotion_launch),
            launch_arguments={
                "start_sdk": start_sdk,
                "require_backend_ready": "False",
                "input_topic": "/cmd_vel",
                "sdk_cmd_vel_topic": "/m20/locomotion/cmd_vel_sdk",
                "execution_profile": "scan_native",
            }.items(),
        ),
        Node(
            package="m20_nav2_system",
            executable="m20_nav2_goal_bridge",
            name="m20_nav2_goal_bridge",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
