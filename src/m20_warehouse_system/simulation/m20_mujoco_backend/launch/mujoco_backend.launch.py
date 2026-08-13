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

"""Generate the selected warehouse world and start its MuJoCo backend."""

from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from m20_mujoco_backend.world_generator import (
    cached_world_path,
    generate_world,
)


def _runtime_actions(context):
    system_config = Path(
        LaunchConfiguration('system_config').perform(context)
    ).expanduser().resolve()
    configured_root = LaunchConfiguration('package_root').perform(context)
    package_root = (
        Path(configured_root).expanduser().resolve()
        if configured_root
        else system_config.parent.parent
    )
    backend_share = Path(
        get_package_share_directory('m20_mujoco_backend')
    )
    description_share = Path(
        get_package_share_directory('m20_official_description')
    )
    output_path = cached_world_path(system_config)
    report = generate_world(
        system_config=system_config,
        package_root=package_root,
        robot_template=backend_share / 'models' / 'm20_robot.xml',
        mesh_directory=description_share / 'meshes',
        output_path=output_path,
        validate_model=True,
    )
    with system_config.open('r', encoding='utf-8') as stream:
        system = yaml.safe_load(stream)
    initial_pose = system['floors']['F1']['initial_pose']

    def initial_component(argument: str, index: int) -> float:
        """Resolve an optional simulation-only cold-start pose override."""
        configured = LaunchConfiguration(argument).perform(context).strip()
        return float(configured) if configured else float(initial_pose[index])

    return [
        Node(
            package='m20_mujoco_backend',
            executable='m20_mujoco_backend',
            name='m20_mujoco_backend',
            output='screen',
            parameters=[
                str(backend_share / 'config' / 'mujoco_backend.yaml'),
                {
                    'model_xml_path': report.output_path,
                    # Keep GUI synchronization outside the 1 kHz physics and
                    # official-policy feedback process.
                    'use_viewer': False,
                    'viewer_distance': LaunchConfiguration(
                        'viewer_distance'
                    ),
                    'viewer_azimuth': LaunchConfiguration(
                        'viewer_azimuth'
                    ),
                    'viewer_elevation': LaunchConfiguration(
                        'viewer_elevation'
                    ),
                    'viewer_max_fps': LaunchConfiguration(
                        'viewer_max_fps'
                    ),
                    'initial_x': initial_component('initial_x', 0),
                    'initial_y': initial_component('initial_y', 1),
                    'initial_yaw': initial_component('initial_yaw', 2),
                    'real_time_factor': LaunchConfiguration(
                        'real_time_factor'
                    ),
                },
            ],
        ),
        Node(
            package='m20_mujoco_backend',
            executable='m20_mujoco_viewer',
            name='m20_mujoco_viewer',
            output='screen',
            # Rendering is diagnostic-only.  Keep it below the official
            # policy and 1 kHz physics processes in the Linux scheduler.
            prefix='nice -n 10',
            condition=IfCondition(LaunchConfiguration('use_viewer')),
            parameters=[
                {
                    'model_xml_path': report.output_path,
                    'distance': LaunchConfiguration('viewer_distance'),
                    'azimuth': LaunchConfiguration('viewer_azimuth'),
                    'elevation': LaunchConfiguration('viewer_elevation'),
                    'max_fps': LaunchConfiguration('viewer_max_fps'),
                    'initial_x': initial_component('initial_x', 0),
                    'initial_y': initial_component('initial_y', 1),
                    'initial_yaw': initial_component('initial_yaw', 2),
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the backend's profile and rendering controls."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'system_config',
                description='Absolute warehouse system YAML path.',
            ),
            DeclareLaunchArgument(
                'package_root',
                default_value='',
                description=(
                    'Root containing map paths in system_config; inferred '
                    'from the configuration install location when empty.'
                ),
            ),
            DeclareLaunchArgument(
                'use_viewer',
                default_value='true',
                description=(
                    'Open the native MuJoCo 3D viewer; set false for '
                    'headless runs.'
                ),
            ),
            DeclareLaunchArgument(
                'viewer_distance',
                default_value='4.0',
                description=(
                    'Robot-centered direct-follow distance in metres.'
                ),
            ),
            DeclareLaunchArgument(
                'viewer_azimuth',
                default_value='90.0',
                description='Initial direct-follow camera azimuth in degrees.',
            ),
            DeclareLaunchArgument(
                'viewer_elevation',
                default_value='-89.0',
                description='Initial near-vertical camera elevation in degrees.',
            ),
            DeclareLaunchArgument(
                'viewer_max_fps',
                default_value='30.0',
                description='Maximum isolated viewer wall-clock frame rate.',
            ),
            DeclareLaunchArgument('real_time_factor', default_value='1.0'),
            DeclareLaunchArgument(
                'initial_x',
                default_value='',
                description=(
                    'Optional MuJoCo cold-start x override. Empty uses the '
                    'F1 initial_pose from system_config.'
                ),
            ),
            DeclareLaunchArgument(
                'initial_y',
                default_value='',
                description=(
                    'Optional MuJoCo cold-start y override. Empty uses the '
                    'F1 initial_pose from system_config.'
                ),
            ),
            DeclareLaunchArgument(
                'initial_yaw',
                default_value='',
                description=(
                    'Optional MuJoCo cold-start yaw override. Empty uses the '
                    'F1 initial_pose from system_config.'
                ),
            ),
            OpaqueFunction(function=_runtime_actions),
        ]
    )
