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

"""Run the official M20 SDK in an obstacle-free MuJoCo test field."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Compose only physics, the official SDK policy, and the test probe."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    backend = Path(get_package_share_directory('m20_mujoco_backend'))
    locomotion = Path(
        get_package_share_directory('m20_locomotion_control')
    )

    use_viewer = LaunchConfiguration('use_viewer')
    run_probe = LaunchConfiguration('run_probe')
    suite = LaunchConfiguration('suite')
    case_name = LaunchConfiguration('case_name')
    output_directory = LaunchConfiguration('output_directory')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'system_config',
                default_value=str(
                    integration / 'config' / 'motion_envelope_system.yaml'
                ),
            ),
            DeclareLaunchArgument('use_viewer', default_value='false'),
            DeclareLaunchArgument('real_time_factor', default_value='1.0'),
            DeclareLaunchArgument(
                'run_probe',
                default_value='true',
                description=(
                    'Run the scripted probe and shut down when it completes.'
                ),
            ),
            DeclareLaunchArgument(
                'suite',
                default_value='smoke',
                description=(
                    'Probe suite: smoke, axial, navigation, reverse, '
                    'turn_matrix, or full.'
                ),
            ),
            DeclareLaunchArgument(
                'case_name',
                default_value='',
                description='Optional single case name, overriding suite.',
            ),
            DeclareLaunchArgument(
                'output_directory',
                default_value='/tmp/m20_motion_envelope',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(backend / 'launch' / 'mujoco_backend.launch.py')
                ),
                launch_arguments={
                    'system_config': LaunchConfiguration('system_config'),
                    'package_root': str(integration),
                    'use_viewer': use_viewer,
                    'real_time_factor': LaunchConfiguration(
                        'real_time_factor'
                    ),
                }.items(),
            ),
            # Bypass both the SCAN controller and project motion adapter.
            # The probe publishes exact Twist commands to this SDK input.
            Node(
                package='m20_sdk_deploy',
                executable='rl_deploy_cmdvel',
                name='m20_cmd_vel_interface',
                output='screen',
                parameters=[
                    str(
                        locomotion
                        / 'config'
                        / 'sdk_locomotion.yaml'
                    ),
                ],
            ),
            Node(
                package='m20_warehouse_inspection',
                executable='m20_command_envelope_probe',
                name='m20_command_envelope_probe',
                output='screen',
                arguments=[
                    '--suite',
                    suite,
                    '--case',
                    case_name,
                    '--output-directory',
                    output_directory,
                ],
                condition=IfCondition(run_probe),
                on_exit=[
                    Shutdown(
                        reason='M20 command-envelope probe completed'
                    )
                ],
            ),
        ]
    )
