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

"""Complete M20 warehouse inspection with MuJoCo as execution backend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Compose SCAN, maps, tasks, official RL control, and M20 physics."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    mujoco_backend = Path(
        get_package_share_directory('m20_mujoco_backend')
    )
    locomotion = Path(
        get_package_share_directory('m20_locomotion_control')
    )
    default_config = (
        integration / 'config' / 'dense_four_corner_system.yaml'
    )
    system_config = LaunchConfiguration('system_config')
    use_rviz = LaunchConfiguration('use_rviz')
    use_planner = LaunchConfiguration('use_planner')
    use_grid_route = LaunchConfiguration('use_grid_route')
    clearance_config = LaunchConfiguration('clearance_config')
    use_mujoco_viewer = LaunchConfiguration('use_mujoco_viewer')
    mujoco_viewer_distance = LaunchConfiguration(
        'mujoco_viewer_distance'
    )
    real_time_factor = LaunchConfiguration('real_time_factor')
    navigation_timeout_sec = LaunchConfiguration(
        'navigation_timeout_sec'
    )
    run_acceptance = LaunchConfiguration('run_acceptance')
    acceptance_mode = LaunchConfiguration('acceptance_mode')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'system_config',
                default_value=str(default_config),
                description=(
                    'Warehouse profile. The 0.90 m dense profile is the '
                    'default complete-system scene.'
                ),
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_planner', default_value='true'),
            DeclareLaunchArgument('use_grid_route', default_value='false'),
            DeclareLaunchArgument(
                'clearance_config',
                default_value=str(
                    Path(
                        get_package_share_directory(
                            'm20_scan_navigation'
                        )
                    )
                    / 'config'
                    / 'clearance_conservative.yaml'
                ),
                description=(
                    'Unified SCAN/collision clearance parameter file.'
                ),
            ),
            DeclareLaunchArgument(
                'use_mujoco_viewer',
                default_value='true',
                description=(
                    'Open the native MuJoCo 3D viewer by default; set false '
                    'for headless runs.'
                ),
            ),
            DeclareLaunchArgument(
                'mujoco_viewer_distance',
                default_value='4.0',
                description=(
                    'MuJoCo robot-follow camera distance in metres.'
                ),
            ),
            DeclareLaunchArgument('real_time_factor', default_value='1.0'),
            DeclareLaunchArgument(
                'navigation_timeout_sec',
                default_value='300.0',
                description=(
                    'Per-step timeout for physical MuJoCo motion; longer '
                    'than the RViz kinematic profile.'
                ),
            ),
            DeclareLaunchArgument('run_acceptance', default_value='false'),
            DeclareLaunchArgument('acceptance_mode', default_value='quick'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        integration
                        / 'launch'
                        / 'inspection_mission_rviz.launch.py'
                    )
                ),
                launch_arguments={
                    'system_config': system_config,
                    'use_rviz': use_rviz,
                    'use_planner': use_planner,
                    'use_grid_route': use_grid_route,
                    'clearance_config': clearance_config,
                    'motion_backend': 'external',
                    'navigation_timeout_sec': navigation_timeout_sec,
                    'run_acceptance': run_acceptance,
                    'acceptance_mode': acceptance_mode,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        mujoco_backend
                        / 'launch'
                        / 'mujoco_backend.launch.py'
                    )
                ),
                launch_arguments={
                    'system_config': system_config,
                    'package_root': str(integration),
                    'use_viewer': use_mujoco_viewer,
                    'viewer_distance': mujoco_viewer_distance,
                    'real_time_factor': real_time_factor,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        locomotion
                        / 'launch'
                        / 'sdk_locomotion.launch.py'
                    )
                ),
                launch_arguments={
                    'start_sdk': 'true',
                    'require_backend_ready': 'true',
                }.items(),
            ),
        ]
    )
