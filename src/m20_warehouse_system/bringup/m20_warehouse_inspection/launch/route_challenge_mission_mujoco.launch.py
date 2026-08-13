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

"""Run the obstacle-interruption profile through the MuJoCo backend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Select route-challenge assets while reusing the full MuJoCo graph."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    system_config = (
        integration / 'config' / 'route_challenge_system.yaml'
    )
    use_rviz = LaunchConfiguration('use_rviz')
    use_planner = LaunchConfiguration('use_planner')
    use_mujoco_viewer = LaunchConfiguration('use_mujoco_viewer')
    mujoco_viewer_distance = LaunchConfiguration(
        'mujoco_viewer_distance'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_planner', default_value='true'),
            DeclareLaunchArgument(
                'use_mujoco_viewer',
                default_value='true',
            ),
            DeclareLaunchArgument(
                'mujoco_viewer_distance',
                default_value='4.0',
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
                    'system_config': str(system_config),
                    'use_rviz': use_rviz,
                    'use_planner': use_planner,
                    'use_mujoco_viewer': use_mujoco_viewer,
                    'mujoco_viewer_distance': mujoco_viewer_distance,
                }.items(),
            ),
        ]
    )
