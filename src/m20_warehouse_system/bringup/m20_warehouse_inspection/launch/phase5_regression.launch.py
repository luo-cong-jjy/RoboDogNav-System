# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch phase-5 stability or fault regression inside the system graph."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Compose production-speed phase 4 with one regression controller."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    default_config = (
        integration / 'config' / 'flat_multifloor_system.yaml'
    )
    system_config = LaunchConfiguration('system_config')
    use_rviz = LaunchConfiguration('use_rviz')
    mode = LaunchConfiguration('mode')
    switch_iterations = LaunchConfiguration('switch_iterations')
    mission_runs = LaunchConfiguration('mission_runs')
    mission_timeout = LaunchConfiguration('mission_timeout_sec')
    memory_growth_limit = LaunchConfiguration('memory_growth_limit_mib')
    memory_slope_limit = LaunchConfiguration(
        'memory_slope_limit_mib'
    )
    regression = Node(
        package='m20_warehouse_inspection',
        executable='m20_phase5_regression',
        name='m20_phase5_regression',
        output='screen',
        parameters=[
            {
                'config_path': system_config,
                'mode': mode,
                'switch_iterations': switch_iterations,
                'mission_runs': mission_runs,
                'mission_timeout_sec': mission_timeout,
                'memory_growth_limit_mib': memory_growth_limit,
                'memory_slope_limit_mib': memory_slope_limit,
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='false'),
            DeclareLaunchArgument(
                'system_config',
                default_value=str(default_config),
                description='Absolute system profile YAML path.',
            ),
            DeclareLaunchArgument(
                'mode', default_value='switch_stress'
            ),
            DeclareLaunchArgument(
                'switch_iterations', default_value='20'
            ),
            DeclareLaunchArgument('mission_runs', default_value='10'),
            DeclareLaunchArgument(
                'mission_timeout_sec', default_value='900.0'
            ),
            DeclareLaunchArgument(
                'memory_growth_limit_mib', default_value='128.0'
            ),
            DeclareLaunchArgument(
                'memory_slope_limit_mib', default_value='8.0'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        integration
                        / 'launch'
                        / 'inspection_mission_rviz.launch.py'
                    )
                ),
                launch_arguments={
                    'use_rviz': use_rviz,
                    'use_planner': 'true',
                    'run_acceptance': 'false',
                    'system_config': system_config,
                }.items(),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=regression,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason='phase-5 regression finished'
                            )
                        )
                    ],
                )
            ),
            regression,
        ]
    )
