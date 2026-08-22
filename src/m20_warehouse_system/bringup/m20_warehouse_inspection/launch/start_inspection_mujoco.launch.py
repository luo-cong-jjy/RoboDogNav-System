"""Trigger an inspection mission against an already running MuJoCo stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    default_config = package_share / 'config' / 'dense_four_corner_system.yaml'
    system_config = LaunchConfiguration('system_config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'system_config',
            default_value=str(default_config),
            description='Mission configuration for the route executor.',
        ),
        DeclareLaunchArgument(
            'mission_id',
            default_value='dense_four_corner_patrol',
            description='Mission identifier configured in the running system.',
        ),
        DeclareLaunchArgument(
            'server_timeout',
            default_value='15.0',
            description='Seconds to wait for the mission action server.',
        ),
        Node(
            package='m20_inspection_core',
            executable='m20_mission_executor',
            name='m20_mission_executor',
            output='screen',
            parameters=[{'config_path': system_config}],
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'm20_warehouse_inspection',
                'm20_start_inspection',
                '--mission-id', LaunchConfiguration('mission_id'),
                '--server-timeout', LaunchConfiguration('server_timeout'),
            ],
            output='screen',
        ),
    ])
