from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_core_launch = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection_mujoco'),
        'launch',
        'control_core.launch.py',
    ])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(control_core_launch)),
    ])
