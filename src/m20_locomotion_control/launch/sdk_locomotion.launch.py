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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    """Start the project-owned intent adapter and optional vendor controller."""
    config = PathJoinSubstitution(
        [FindPackageShare('m20_locomotion_control'), 'config',
         'sdk_locomotion.yaml']
    )
    start_sdk = LaunchConfiguration('start_sdk')
    require_backend_ready = LaunchConfiguration('require_backend_ready')
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
            Node(
                package='m20_locomotion_control',
                executable='m20_locomotion_manager',
                name='m20_locomotion_manager',
                output='screen',
                parameters=[
                    config,
                    {'require_backend_ready': require_backend_ready},
                ],
            ),
            Node(
                package='m20_sdk_deploy',
                executable='rl_deploy_cmdvel',
                output='screen',
                parameters=[config],
                condition=IfCondition(start_sdk),
            ),
        ]
    )
