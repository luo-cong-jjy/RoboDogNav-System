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
                    'use_viewer': LaunchConfiguration('use_viewer'),
                    'viewer_distance': LaunchConfiguration(
                        'viewer_distance'
                    ),
                    'initial_x': float(initial_pose[0]),
                    'initial_y': float(initial_pose[1]),
                    'initial_yaw': float(initial_pose[2]),
                    'real_time_factor': LaunchConfiguration(
                        'real_time_factor'
                    ),
                },
            ],
        )
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
                    'Robot-centered tracking-camera distance in metres.'
                ),
            ),
            DeclareLaunchArgument('real_time_factor', default_value='1.0'),
            OpaqueFunction(function=_runtime_actions),
        ]
    )
