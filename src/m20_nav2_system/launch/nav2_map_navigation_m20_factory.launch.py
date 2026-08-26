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
    nav2_params = str(pkg_share / "config" / "nav2_params.yaml")
    default_map = str(pkg_share / "maps" / "factory" / "factory_slam_map.yaml")
    rviz_config = str(pkg_share / "rviz" / "nav2_sandbox.rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    navigation_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_delay", default_value="8.0"),
        DeclareLaunchArgument("params_file", default_value=nav2_params),
        DeclareLaunchArgument("map", default_value=default_map),
        # Load and activate the latched map before costmaps are created.
        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            output="screen", parameters=[params_file, {
                "use_sim_time": use_sim_time, "yaml_filename": map_file,
            }],
        ),
        TimerAction(period=2.0, actions=[Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_map", output="screen", parameters=[{
                "use_sim_time": use_sim_time, "autostart": True,
                "node_names": ["map_server"],
            }],
        )]),
        # Navigation only: the bridge supplies map->odom and the odom adapter
        # supplies odom->base_link, so AMCL must not create another TF source.
        TimerAction(period=4.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch), launch_arguments={
                "use_sim_time": use_sim_time, "params_file": params_file,
                "autostart": "True", "use_composition": "False",
                "container_name": "m20_nav2_container",
            }.items(),
        )]),
        TimerAction(period=LaunchConfiguration("rviz_delay"), actions=[Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", rviz_config], parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
        )]),
    ])
