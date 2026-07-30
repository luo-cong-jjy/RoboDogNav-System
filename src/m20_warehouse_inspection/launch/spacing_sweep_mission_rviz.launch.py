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

"""Launch one controlled obstacle-spacing experiment profile."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


PROFILE_TAGS = {
    '0.70': '070',
    '0.80': '080',
    '0.90': '090',
    '1.00': '100',
}


def _include_profile(context):
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    spacing = LaunchConfiguration('spacing').perform(context)
    tag = PROFILE_TAGS.get(spacing)
    if tag is None:
        choices = ', '.join(PROFILE_TAGS)
        raise RuntimeError(
            f'unsupported spacing {spacing!r}; choose one of: {choices}'
        )
    config = integration / 'config' / (
        f'spacing_sweep_{tag}_system.yaml'
    )
    if not config.is_file():
        raise RuntimeError(
            f'missing generated spacing profile: {config}; regenerate it '
            'with tools/generate_spacing_sweep.py'
        )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    integration
                    / 'launch'
                    / 'inspection_mission_rviz.launch.py'
                )
            ),
            launch_arguments={
                'system_config': str(config),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_planner': LaunchConfiguration('use_planner'),
                'use_grid_route': LaunchConfiguration('use_grid_route'),
                'run_acceptance': LaunchConfiguration('run_acceptance'),
                'acceptance_mode': LaunchConfiguration('acceptance_mode'),
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Select one spacing while preserving the production node graph."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'spacing',
                default_value='0.70',
                choices=list(PROFILE_TAGS),
                description='Obstacle body-to-body minimum spacing in metres.',
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_planner', default_value='true'),
            DeclareLaunchArgument('use_grid_route', default_value='false'),
            DeclareLaunchArgument('run_acceptance', default_value='false'),
            DeclareLaunchArgument('acceptance_mode', default_value='quick'),
            OpaqueFunction(function=_include_profile),
        ]
    )
