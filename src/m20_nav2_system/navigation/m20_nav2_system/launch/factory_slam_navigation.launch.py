"""Run Gazebo 2D sensors, slam_toolbox, and Nav2 without MuJoCo."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = Path(get_package_share_directory("m20_nav2_system"))
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("m20_nav2_system"), "launch",
        "gazebo_sensor_m20_factory_2d_scan.launch.py",
    ])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py",
    ])
    slam_params = str(share / "config" / "slam_toolbox.yaml")
    nav2_params = str(share / "config" / "nav2_params.yaml")
    rviz_config = str(share / "rviz" / "nav2_sandbox.rviz")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_keyboard = LaunchConfiguration("use_keyboard")
    mapping_world = str(share / "worlds" / "factory_environment_mapping.world")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_keyboard", default_value="true"),
        DeclareLaunchArgument("use_gazebo_gui", default_value="true"),
        DeclareLaunchArgument("gazebo_gui_delay", default_value="1.0"),
        DeclareLaunchArgument("spawn_delay", default_value="4.0"),
        DeclareLaunchArgument("rviz_delay", default_value="3.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "gazebo_use_rviz": "false",
                "use_gazebo_gui": LaunchConfiguration("use_gazebo_gui"),
                "gazebo_gui_delay": LaunchConfiguration("gazebo_gui_delay"),
                "spawn_delay": LaunchConfiguration("spawn_delay"),
                "world": mapping_world,
            }.items(),
        ),
        # Build /map and map->odom from Gazebo /scan; no pre-existing map is used.
        TimerAction(period=6.0, actions=[Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time, "mode": "mapping"}],
            remappings=[("scan", "/scan")],
        )]),
        # navigation_launch starts Nav2 servers only; slam_toolbox supplies map->odom.
        TimerAction(period=8.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": "true",
                "use_composition": "False",
                "params_file": nav2_params,
            }.items(),
        )]),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
        Node(
            package="m20_nav2_system",
            executable="m20_keyboard_teleop",
            name="m20_keyboard_teleop",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_keyboard),
        ),
    ])
