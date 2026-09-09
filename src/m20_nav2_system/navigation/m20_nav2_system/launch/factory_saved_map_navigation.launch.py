"""Run Gazebo with a saved 2D map, AMCL localization, and Nav2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = Path(get_package_share_directory("m20_nav2_system"))
    gazebo = PathJoinSubstitution([
        FindPackageShare("m20_nav2_system"), "launch",
        "gazebo_sensor_m20_factory_2d_scan.launch.py",
    ])
    nav = PathJoinSubstitution([
        FindPackageShare("m20_nav2_system"), "launch",
        "nav2_map_navigation_m20_factory.launch.py",
    ])
    default_map = str(share / "maps" / "factory" / "m20_factory_slam.yaml")
    # Gazebo owns this dynamic world; the other backend has its own world.
    dynamic_navigation_world = str(share / "worlds" / "factory_environment_gazebo.world")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gazebo_gui", default_value="true"),
        DeclareLaunchArgument("world", default_value=dynamic_navigation_world),
        LogInfo(msg=[
            "[m20_nav2_system] Gazebo saved-map dynamic world: ",
            LaunchConfiguration("world"),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo),
            launch_arguments={
                "use_sim_time": "true", "gazebo_use_rviz": "false",
                "enable_dynamic_tracker": "true",
                "use_gazebo_gui": LaunchConfiguration("use_gazebo_gui"),
                "world": LaunchConfiguration("world"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "use_rviz": "false", "use_sim_time": "true",
            }.items(),
        ),
    ])
