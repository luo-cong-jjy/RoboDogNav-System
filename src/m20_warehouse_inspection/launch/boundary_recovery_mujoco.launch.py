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

"""Cold-start one fixed M20 warehouse boundary-recovery case."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CASES = {
    # F2 (32.8, 16.8)->(4, 16) translated by -40 m onto its F1 replica.
    'upper_boundary': {
        'initial': (-7.2, 16.8, -3.11),
        'target': (-36.0, 16.0, 0.0),
    },
    # The prior full mission reached this slightly positive-x gateway pose.
    'shared_origin_exit': {
        'initial': (0.10, 0.12, -2.63),
        'target': (-37.0, 0.0, 0.0),
    },
    # Reserved start aisle: the position goal is one metre directly behind
    # an east-facing body. It must be reached by straight reverse tracking,
    # without interpreting the clicked quaternion as a required U-turn.
    'rear_corridor': {
        'initial': (-36.0, 0.0, 0.0),
        'target': (-37.0, 0.0, 0.0),
    },
    # Reproduces the live manual-goal trap observed at (-34.38, 2.30): the
    # aisle is wide enough for straight rolling motion but not a turn.  With
    # use_grid_route=true the route must turn in open space before entering.
    'narrow_corridor_entry': {
        'initial': (-37.0, 0.0, 0.0),
        'target': (-30.62, 3.18, 0.0),
    },
}


def _runtime_actions(context):
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    case_name = LaunchConfiguration('case').perform(context).strip()
    if case_name not in CASES:
        raise ValueError(
            f'unknown boundary case {case_name!r}; choose {sorted(CASES)}'
        )
    case = CASES[case_name]
    initial_x, initial_y, initial_yaw = case['initial']
    target_x, target_y, target_yaw = case['target']
    system_config = LaunchConfiguration('system_config').perform(context)

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
                'system_config': system_config,
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_mujoco_viewer': LaunchConfiguration(
                    'use_mujoco_viewer'
                ),
                'real_time_factor': LaunchConfiguration('real_time_factor'),
                'navigation_timeout_sec': LaunchConfiguration('timeout_sec'),
                'use_grid_route': LaunchConfiguration('use_grid_route'),
                'initial_x': str(initial_x),
                'initial_y': str(initial_y),
                'initial_yaw': str(initial_yaw),
            }.items(),
        ),
        Node(
            package='m20_warehouse_inspection',
            executable='m20_boundary_recovery_probe',
            name=f'm20_boundary_recovery_{case_name}',
            output='screen',
            arguments=[
                '--case',
                case_name,
                '--system-config',
                system_config,
                '--output-directory',
                LaunchConfiguration('output_directory'),
                '--target-x',
                str(target_x),
                '--target-y',
                str(target_y),
                '--target-yaw',
                str(target_yaw),
                '--timeout-sec',
                LaunchConfiguration('timeout_sec'),
            ],
            on_exit=[
                Shutdown(reason=f'boundary recovery {case_name} completed')
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the fixed case and evidence output controls."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'case',
                default_value='upper_boundary',
                description=(
                    'upper_boundary, shared_origin_exit, rear_corridor, or '
                    'narrow_corridor_entry'
                ),
            ),
            DeclareLaunchArgument(
                'system_config',
                default_value=str(
                    integration
                    / 'config'
                    / 'dense_four_corner_system.yaml'
                ),
            ),
            DeclareLaunchArgument('use_rviz', default_value='false'),
            DeclareLaunchArgument('use_grid_route', default_value='false'),
            DeclareLaunchArgument(
                'use_mujoco_viewer', default_value='false'
            ),
            DeclareLaunchArgument('real_time_factor', default_value='1.0'),
            DeclareLaunchArgument('timeout_sec', default_value='240.0'),
            DeclareLaunchArgument(
                'output_directory',
                default_value='/tmp/m20_boundary_recovery',
            ),
            OpaqueFunction(function=_runtime_actions),
        ]
    )
