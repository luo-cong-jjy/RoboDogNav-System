from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default_params = str(
        Path(get_package_share_directory("m20_lidar_bridge"))
        / "config"
        / "aos_roi_bridge.yaml"
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to ROI bridge parameters.",
            ),
            Node(
                package="m20_lidar_bridge",
                executable="pointcloud_roi_bridge",
                name="pointcloud_roi_bridge",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
