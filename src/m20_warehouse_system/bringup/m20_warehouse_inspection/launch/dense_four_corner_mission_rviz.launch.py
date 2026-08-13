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

"""Launch the independent high-density four-corner patrol profile."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Select the dense profile while reusing the complete mission graph."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    system_config = (
        integration / 'config' / 'dense_four_corner_system.yaml'
    )
    use_rviz = LaunchConfiguration('use_rviz')
    use_planner = LaunchConfiguration('use_planner')
    run_acceptance = LaunchConfiguration('run_acceptance')
    acceptance_mode = LaunchConfiguration('acceptance_mode')
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_planner', default_value='true'),
            DeclareLaunchArgument('run_acceptance', default_value='false'),
            DeclareLaunchArgument('acceptance_mode', default_value='quick'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        integration
                        / 'launch'
                        / 'inspection_mission_rviz.launch.py'
                    )
                ),
                launch_arguments={
                    'system_config': str(system_config),
                    'use_rviz': use_rviz,
                    'use_planner': use_planner,
                    'run_acceptance': run_acceptance,
                    'acceptance_mode': acceptance_mode,
                }.items(),
            ),
        ]
    )
