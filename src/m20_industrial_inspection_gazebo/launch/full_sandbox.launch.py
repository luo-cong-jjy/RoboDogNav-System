from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("m20_industrial_inspection_gazebo"),
        "launch",
        "gazebo_sensor_sandbox.launch.py",
    ])

    nav2_launch = PathJoinSubstitution([
        FindPackageShare("m20_industrial_inspection_gazebo"),
        "launch",
        "nav2_slam_sandbox.launch.py",
    ])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch)),
    ])
