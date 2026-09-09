"""Deterministic map-based Nav2 launch for the M20 factory simulation."""

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
    pkg_share = Path(get_package_share_directory("m20_nav2_system"))
    # Gazebo owns this profile; the other backend has its own profile.
    nav2_params = str(pkg_share / "config" / "nav2_params_gazebo.yaml")
    default_map = str(pkg_share / "maps" / "factory" / "factory_slam_map.yaml")
    rviz_config = str(pkg_share / "rviz" / "nav2_sandbox_gazebo.rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    bringup_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_delay", default_value="3.0"),
        DeclareLaunchArgument("params_file", default_value=nav2_params),
        DeclareLaunchArgument("map", default_value=default_map),
        # Saved-map navigation requires AMCL to publish map->odom. Use the
        # standard Nav2 bringup so map_server, AMCL and navigation lifecycle
        # nodes are started as one coherent stack.
        # Start Nav2 after Gazebo has spawned the four-wheel model and its
        # odom->base_link TF; otherwise costmaps can remain stuck waiting for
        # a transform that did not exist during lifecycle activation.
        TimerAction(period=6.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch), launch_arguments={
                "slam": "False", "map": map_file,
                "use_sim_time": use_sim_time, "params_file": params_file,
                "autostart": "True", "use_composition": "False",
            }.items(),
        )]),
        TimerAction(period=LaunchConfiguration("rviz_delay"), actions=[Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", rviz_config], parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
        )]),
    ])
