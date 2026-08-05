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

"""Connect the safe warehouse command stream to the official M20 RL SDK."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from m20_locomotion_control.capability_profile import (
    load_capability_profile,
)


def _runtime_actions(context):
    """Resolve and validate one capability profile before starting nodes."""
    share = Path(get_package_share_directory('m20_locomotion_control'))
    config = str(share / 'config' / 'sdk_locomotion.yaml')
    profile_path = LaunchConfiguration(
        'locomotion_capability_config'
    ).perform(context)
    profile = load_capability_profile(profile_path)
    require_backend_ready = LaunchConfiguration('require_backend_ready')
    return [
        Node(
            package='m20_locomotion_control',
            executable='m20_locomotion_manager',
            name='m20_locomotion_manager',
            output='screen',
            parameters=[
                config,
                profile.intent_parameters(),
                {'require_backend_ready': require_backend_ready},
            ],
        ),
        Node(
            package='m20_sdk_deploy',
            executable='rl_deploy_cmdvel',
            output='screen',
            parameters=[config, profile.sdk_parameters()],
            condition=IfCondition(LaunchConfiguration('start_sdk')),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Start the project-owned intent adapter and optional vendor controller."""
    share = Path(get_package_share_directory('m20_locomotion_control'))
    default_capability = (
        share / 'config' / 'm20_policy_v1_capabilities.yaml'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_sdk',
                default_value='false',
                description=(
                    'Start m20_sdk_deploy/rl_deploy_cmdvel. Enable only when '
                    'a MuJoCo, Gazebo joint bridge, or real M20 backend is live.'
                ),
            ),
            DeclareLaunchArgument(
                'require_backend_ready',
                default_value='false',
                description=(
                    'Gate SDK velocity commands on the MuJoCo ready/fault '
                    'contract.'
                ),
            ),
            DeclareLaunchArgument(
                'locomotion_capability_config',
                default_value=str(default_capability),
                description=(
                    'Versioned M20 command, turn and recovery capability '
                    'profile.'
                ),
            ),
            OpaqueFunction(function=_runtime_actions),
        ]
    )
