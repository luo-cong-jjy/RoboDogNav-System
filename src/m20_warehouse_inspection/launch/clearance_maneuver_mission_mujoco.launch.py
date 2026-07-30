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

"""Launch the single-obstacle double-bypass MuJoCo benchmark."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Compose the benchmark with production conservative clearance."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    navigation = Path(
        get_package_share_directory('m20_scan_navigation')
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'system_config',
                default_value=str(
                    integration
                    / 'config'
                    / 'clearance_maneuver_system.yaml'
                ),
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument(
                'use_mujoco_viewer',
                default_value='true',
            ),
            DeclareLaunchArgument(
                'navigation_timeout_sec',
                default_value='120.0',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        integration
                        / 'launch'
                        / 'inspection_mission_mujoco.launch.py'
                    )
                ),
                launch_arguments={
                    'system_config': LaunchConfiguration('system_config'),
                    'clearance_config': str(
                        navigation
                        / 'config'
                        / 'clearance_conservative.yaml'
                    ),
                    'use_rviz': LaunchConfiguration('use_rviz'),
                    'use_mujoco_viewer': LaunchConfiguration(
                        'use_mujoco_viewer'
                    ),
                    'use_planner': 'true',
                    'use_grid_route': 'false',
                    'navigation_timeout_sec': LaunchConfiguration(
                        'navigation_timeout_sec'
                    ),
                }.items(),
            ),
        ]
    )
