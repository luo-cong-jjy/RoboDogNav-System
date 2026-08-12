# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the phase-1 flat F1/F2 map baseline and optional RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the phase-1 launch graph."""
    package_root = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    default_config = package_root / 'config' / 'flat_multifloor_system.yaml'
    default_rviz = package_root / 'rviz' / 'flat_multifloor.rviz'

    config_path = LaunchConfiguration('config_path')
    initial_floor = LaunchConfiguration('initial_floor')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_path',
                default_value=str(default_config),
                description='System-level flat multi-floor configuration',
            ),
            DeclareLaunchArgument(
                'initial_floor',
                default_value='F1',
                description='Only this floor is published to active-map topics',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Start RViz for the two-region overview',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=str(default_rviz),
                description='RViz configuration file',
            ),
            Node(
                package='m20_warehouse_inspection',
                executable='m20_flat_map_server',
                name='flat_multifloor_map_server',
                output='screen',
                parameters=[
                    {
                        'config_path': config_path,
                        'package_root': str(package_root),
                        'initial_floor': initial_floor,
                    }
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='flat_multifloor_rviz',
                arguments=['-d', rviz_config],
                output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]
    )
