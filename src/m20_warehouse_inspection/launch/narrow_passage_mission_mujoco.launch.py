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

"""Launch one controlled M20 doorway-width test in the full MuJoCo graph."""

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


WIDTH_TAGS = {
    '0.60': '060',
    '0.65': '065',
    '0.70': '070',
    '0.75': '075',
    '0.80': '080',
    '0.90': '090',
}
CLEARANCE_PROFILES = (
    'vendor',
    'tight',
    'balanced',
    'conservative',
)


def _include_profile(context):
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    navigation = Path(
        get_package_share_directory('m20_scan_navigation')
    )
    width = LaunchConfiguration('width').perform(context)
    tag = WIDTH_TAGS.get(width)
    if tag is None:
        raise RuntimeError(
            f'unsupported width {width!r}; choose one of '
            f'{", ".join(WIDTH_TAGS)}'
        )
    clearance = LaunchConfiguration('clearance_profile').perform(context)
    if clearance not in CLEARANCE_PROFILES:
        raise RuntimeError(
            f'unsupported clearance profile {clearance!r}; choose one of '
            f'{", ".join(CLEARANCE_PROFILES)}'
        )
    system_config = (
        integration / 'config' / f'narrow_passage_{tag}_system.yaml'
    )
    clearance_config = (
        navigation / 'config' / f'clearance_{clearance}.yaml'
    )
    for path in (system_config, clearance_config):
        if not path.is_file():
            raise RuntimeError(f'missing narrow-passage test input: {path}')
    return [
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
                'clearance_config': str(clearance_config),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_planner': 'true',
                'use_grid_route': LaunchConfiguration('use_grid_route'),
                'use_mujoco_viewer': LaunchConfiguration(
                    'use_mujoco_viewer'
                ),
                'mujoco_viewer_distance': LaunchConfiguration(
                    'mujoco_viewer_distance'
                ),
                'navigation_timeout_sec': LaunchConfiguration(
                    'navigation_timeout_sec'
                ),
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Select width and clearance without changing the production graph."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'width',
                default_value='0.80',
                choices=list(WIDTH_TAGS),
                description='Nominal obstacle-surface doorway width.',
            ),
            DeclareLaunchArgument(
                'clearance_profile',
                default_value='balanced',
                choices=list(CLEARANCE_PROFILES),
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_grid_route', default_value='false'),
            DeclareLaunchArgument(
                'use_mujoco_viewer',
                default_value='true',
            ),
            DeclareLaunchArgument(
                'mujoco_viewer_distance',
                default_value='4.0',
            ),
            DeclareLaunchArgument(
                'navigation_timeout_sec',
                default_value='120.0',
            ),
            OpaqueFunction(function=_include_profile),
        ]
    )
