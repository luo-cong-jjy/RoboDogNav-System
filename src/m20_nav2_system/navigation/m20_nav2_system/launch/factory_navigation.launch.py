"""Default Gazebo navigation entry using the saved 2D map and AMCL."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    saved_navigation = PathJoinSubstitution([
        FindPackageShare("m20_nav2_system"), "launch",
        "factory_saved_map_navigation.launch.py",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("m20_nav2_system"), "rviz", "nav2_sandbox.rviz",
    ])
    default_world = PathJoinSubstitution([
        FindPackageShare("m20_nav2_system"), "worlds",
        "factory_environment.world",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gazebo_gui", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world),
        LogInfo(msg=[
            "[m20_nav2_system] Gazebo navigation shared world: ",
            LaunchConfiguration("world"),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(saved_navigation),
            launch_arguments={
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_gazebo_gui": LaunchConfiguration("use_gazebo_gui"),
                "world": LaunchConfiguration("world"),
            }.items(),
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", rviz_config], parameters=[{"use_sim_time": True}],
        ),
    ])
