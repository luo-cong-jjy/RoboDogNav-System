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

"""Start the real M20 factory motion backend behind the common safe Twist."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from m20_locomotion_control.capability_profile import (
    load_capability_profile,
)


def _runtime_actions(context):
    transport = LaunchConfiguration('factory_transport').perform(context)
    if transport not in {'basic_server', 'direct_ros'}:
        raise RuntimeError(
            'factory_transport must be basic_server or direct_ros'
        )
    config_path = LaunchConfiguration('factory_motion_config').perform(
        context
    )
    if not config_path:
        share = Path(get_package_share_directory('m20_locomotion_control'))
        filename = (
            'm20_factory_basic_server.yaml'
            if transport == 'basic_server'
            else 'm20_factory_direct_ros.yaml'
        )
        config_path = str(share / 'config' / filename)
    profile = load_capability_profile(
        LaunchConfiguration('locomotion_capability_config').perform(context)
    )
    overrides = {
        'command_ownership_confirmed': ParameterValue(
            LaunchConfiguration('command_ownership_confirmed'),
            value_type=bool,
        ),
        'auto_enable_motion': ParameterValue(
            LaunchConfiguration('auto_enable_motion'), value_type=bool
        ),
    }
    executable = (
        'm20_basic_server_backend'
        if transport == 'basic_server'
        else 'm20_direct_ros_backend'
    )
    if transport == 'basic_server':
        overrides['robot_host'] = LaunchConfiguration('robot_host')
    return [
        Node(
            package='m20_locomotion_control',
            executable='m20_locomotion_manager',
            name='m20_locomotion_manager',
            output='screen',
            parameters=[
                config_path,
                profile.intent_parameters(),
                {'require_backend_ready': True},
            ],
        ),
        Node(
            package='m20_locomotion_control',
            executable=executable,
            name=executable,
            output='screen',
            parameters=[config_path, overrides],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the intentionally guarded real-hardware launch contract."""
    share = Path(get_package_share_directory('m20_locomotion_control'))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'factory_transport',
                default_value='basic_server',
                description=(
                    'Factory transport selected behind the common safe '
                    'Twist contract: basic_server or direct_ros.'
                ),
            ),
            DeclareLaunchArgument(
                'factory_motion_config',
                default_value='',
                description=(
                    'Optional backend YAML override. Empty selects the '
                    'transport-specific packaged profile.'
                ),
            ),
            DeclareLaunchArgument(
                'locomotion_capability_config',
                default_value=str(
                    share
                    / 'config'
                    / 'm20_factory_agile_flat_capabilities.yaml'
                ),
                description='M20 factory-gait navigation capability profile.',
            ),
            DeclareLaunchArgument(
                'robot_host',
                default_value='10.21.31.103',
                description='M20 AOS basic_server address.',
            ),
            DeclareLaunchArgument(
                'command_ownership_confirmed',
                default_value='false',
                description=(
                    'Set true only after the onboard planner and autonomous '
                    'charging motion sources have been stopped.'
                ),
            ),
            DeclareLaunchArgument(
                'auto_enable_motion',
                default_value='false',
                description=(
                    'Normally leave false and enable through '
                    '/m20/hardware/enable_motion after the preflight check.'
                ),
            ),
            OpaqueFunction(function=_runtime_actions),
        ]
    )
