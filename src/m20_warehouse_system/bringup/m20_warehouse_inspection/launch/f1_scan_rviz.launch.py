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

"""Launch the complete phase-2 F1 RViz navigation system."""

from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Connect map, M20 model, sensing, SCAN, safety, and RViz."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    description = Path(
        get_package_share_directory('m20_official_description')
    )
    scan = Path(get_package_share_directory('m20_scan_navigation'))
    scan_vendor = Path(get_package_share_directory('m20_scan_planner'))
    simulation = Path(get_package_share_directory('m20_warehouse_sim'))
    core = Path(get_package_share_directory('m20_inspection_core'))
    locomotion = Path(
        get_package_share_directory('m20_locomotion_control')
    )

    system_config = integration / 'config' / 'flat_multifloor_system.yaml'
    with system_config.open('r', encoding='utf-8') as stream:
        initial_pose = yaml.safe_load(stream)['floors']['F1']['initial_pose']
    model = description / 'urdf' / 'm20_official.urdf'
    # Keep the lightweight F1 entry on the same RViz contract as the native
    # SCAN stack.  The former phase-2 file still referenced retired /m20/*
    # aliases, so its goal tool and most planner displays had no subscribers.
    rviz = scan_vendor / 'rviz' / 'default.rviz'
    use_rviz = LaunchConfiguration('use_rviz')
    use_local_sensing = LaunchConfiguration('use_local_sensing')
    use_planner = LaunchConfiguration('use_planner')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_local_sensing', default_value='true'),
            DeclareLaunchArgument('use_planner', default_value='true'),
            Node(
                package='m20_warehouse_inspection',
                executable='m20_flat_map_server',
                name='flat_multifloor_map_server',
                output='screen',
                parameters=[
                    {
                        'config_path': str(system_config),
                        'package_root': str(integration),
                        'initial_floor': 'F1',
                        # Transient-local consumers receive one snapshot.
                        # Avoid rebuilding inflated maps every ten seconds.
                        'republish_period_sec': 0.0,
                    }
                ],
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='m20_robot_state_publisher',
                output='screen',
                parameters=[
                    {
                        # Direct official M20 geometry/kinematic tree. The
                        # simulated lidar remains a data source, not a model.
                        'robot_description': model.read_text(encoding='utf-8')
                    }
                ],
            ),
            # SCAN's simulator publishes local sensing in "world". During the
            # flat F1 phase, world and map intentionally share one origin.
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='m20_map_to_scan_world',
                # Positional arguments work on both Foxy and Humble.  Foxy
                # does not implement Humble's named-argument CLI.
                arguments=[
                    '0', '0', '0', '0', '0', '0',
                    'map', 'world',
                ],
                output='screen',
            ),
            Node(
                package='m20_warehouse_sim',
                executable='m20_rviz_kinematic_backend',
                output='screen',
                parameters=[
                    str(simulation / 'config' / 'rviz_kinematic.yaml'),
                    {
                        'initial_x': float(initial_pose[0]),
                        'initial_y': float(initial_pose[1]),
                        'initial_yaw': float(initial_pose[2]),
                    },
                ],
            ),
            Node(
                package='m20_locomotion_control',
                executable='m20_navigation_adapter',
                name='m20_navigation_adapter',
                output='screen',
                parameters=[
                    str(locomotion / 'config' / 'sdk_locomotion.yaml')
                ],
            ),
            Node(
                package='m20_inspection_core',
                executable='m20_safety_supervisor',
                output='screen',
                parameters=[str(core / 'config' / 'safety.yaml')],
            ),
            Node(
                package='m20_inspection_core',
                executable='m20_collision_guard',
                name='m20_collision_guard',
                output='screen',
                parameters=[
                    str(
                        core
                        / 'config'
                        / 'collision_guard_scan_native.yaml'
                    )
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(scan / 'launch' / 'f1_scan.launch.py')
                ),
                launch_arguments={
                    'use_local_sensing': use_local_sensing,
                    'use_planner': use_planner,
                }.items(),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='m20_phase2_rviz',
                arguments=['-d', str(rviz)],
                output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]
    )
