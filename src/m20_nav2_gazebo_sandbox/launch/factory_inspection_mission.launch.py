from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("m20_nav2_gazebo_sandbox"))
    default_params = str(pkg_share / "config" / "factory_inspection_midpoints.yaml")

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),

        Node(
            package="m20_nav2_gazebo_sandbox",
            executable="factory_inspection_nav2_mission",
            name="factory_inspection_nav2_mission",
            output="screen",
            parameters=[params_file],
        ),
    ])
