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

"""Launch phase 4: automatic flat multi-floor warehouse inspection."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Compose phase-3 navigation with the typed mission executor."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    default_config = (
        integration / 'config' / 'flat_multifloor_system.yaml'
    )
    system_config = LaunchConfiguration('system_config')
    use_rviz = LaunchConfiguration('use_rviz')
    use_planner = LaunchConfiguration('use_planner')
    use_local_sensing = LaunchConfiguration('use_local_sensing')
    clearance_config = LaunchConfiguration('clearance_config')
    planner_config = LaunchConfiguration('planner_config')
    controller_config = LaunchConfiguration('controller_config')
    motion_backend = LaunchConfiguration('motion_backend')
    execution_profile = LaunchConfiguration('execution_profile')
    body_pose_topic = LaunchConfiguration('body_pose_topic')
    navigation_cloud_topic = LaunchConfiguration(
        'navigation_cloud_topic'
    )
    sensor_pose_topic = LaunchConfiguration('sensor_pose_topic')
    relocation_service = LaunchConfiguration('relocation_service')
    velocity_feedback_enabled = LaunchConfiguration(
        'velocity_feedback_enabled'
    )
    velocity_feedback_source = LaunchConfiguration(
        'velocity_feedback_source'
    )
    velocity_feedback_twist_topic = LaunchConfiguration(
        'velocity_feedback_twist_topic'
    )
    locomotion_capability_config = LaunchConfiguration(
        'locomotion_capability_config'
    )
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
                description='Absolute system profile YAML path.',
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_planner', default_value='true'),
            DeclareLaunchArgument('use_local_sensing', default_value='true'),
            DeclareLaunchArgument(
                'body_pose_topic', default_value='/m20/sim/body_pose'
            ),
            DeclareLaunchArgument(
                'navigation_cloud_topic',
                default_value='/quad_0/cloud',
            ),
            DeclareLaunchArgument(
                'sensor_pose_topic',
                default_value='/quad_0/lidar_pose',
            ),
            DeclareLaunchArgument(
                'relocation_service',
                default_value='/m20/sim/set_pose',
            ),
            DeclareLaunchArgument(
                'clearance_config',
                default_value=str(
                    Path(
                        get_package_share_directory(
                            'm20_scan_navigation'
                        )
                    )
                    / 'config'
                    / 'clearance_m20.yaml'
                ),
                description=(
                    'M20 geometry override for native SCAN planning and the '
                    'optional independent command guard.'
                ),
            ),
            DeclareLaunchArgument(
                'planner_config',
                default_value=str(
                    Path(
                        get_package_share_directory(
                            'm20_scan_navigation'
                        )
                    )
                    / 'config'
                    / 'scan_m20_velocity_planner.yaml'
                ),
                description=(
                    'Velocity-only M20 overlay layered after the upstream '
                    'SCAN planner profile.'
                ),
            ),
            DeclareLaunchArgument(
                'controller_config',
                default_value=str(
                    Path(
                        get_package_share_directory(
                            'm20_scan_navigation'
                        )
                    )
                    / 'config'
                    / 'scan_m20_velocity_controller.yaml'
                ),
                description=(
                    'Velocity-only M20 overlay layered after the upstream '
                    'SCAN closed-loop controller profile.'
                ),
            ),
            DeclareLaunchArgument(
                'motion_backend',
                default_value='rviz',
                description='rviz planar backend or an external backend.',
            ),
            DeclareLaunchArgument(
                'execution_profile',
                default_value='scan_native',
                description=(
                    'Default upstream-compatible SCAN execution; use '
                    'm20_safe for the Twist guard/adapter or m20_progress '
                    'for measured-progress B-spline execution.'
                ),
            ),
            DeclareLaunchArgument(
                'velocity_feedback_enabled',
                default_value='false',
                description=(
                    'Enable bounded measured body-velocity PI compensation.'
                ),
            ),
            DeclareLaunchArgument(
                'velocity_feedback_source',
                default_value='odometry',
            ),
            DeclareLaunchArgument(
                'velocity_feedback_twist_topic',
                default_value='/m20/locomotion/measured_twist',
            ),
            DeclareLaunchArgument(
                'locomotion_capability_config',
                default_value=str(
                    Path(
                        get_package_share_directory(
                            'm20_locomotion_control'
                        )
                    )
                    / 'config'
                    / 'm20_policy_v1_capabilities.yaml'
                ),
                description='Versioned M20 platform capability profile.',
            ),
            DeclareLaunchArgument(
                'navigation_timeout_sec',
                default_value='180.0',
                description='Per-navigation-step mission timeout.',
            ),
            DeclareLaunchArgument('run_acceptance', default_value='false'),
            DeclareLaunchArgument('acceptance_mode', default_value='quick'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        integration
                        / 'launch'
                        / 'multifloor_scan_rviz.launch.py'
                    )
                ),
                launch_arguments={
                    'use_rviz': use_rviz,
                    'use_planner': use_planner,
                    'use_local_sensing': use_local_sensing,
                    'body_pose_topic': body_pose_topic,
                    'navigation_cloud_topic': navigation_cloud_topic,
                    'sensor_pose_topic': sensor_pose_topic,
                    'relocation_service': relocation_service,
                    'clearance_config': clearance_config,
                    'planner_config': planner_config,
                    'controller_config': controller_config,
                    'motion_backend': motion_backend,
                    'execution_profile': execution_profile,
                    'velocity_feedback_enabled': velocity_feedback_enabled,
                    'velocity_feedback_source': velocity_feedback_source,
                    'velocity_feedback_twist_topic': (
                        velocity_feedback_twist_topic
                    ),
                    'locomotion_capability_config': (
                        locomotion_capability_config
                    ),
                    'system_config': system_config,
                }.items(),
            ),
            Node(
                package='m20_inspection_core',
                executable='m20_mission_executor',
                name='m20_mission_executor',
                output='screen',
                parameters=[
                    {
                        'config_path': system_config,
                        'navigation_timeout_sec': ParameterValue(
                            navigation_timeout_sec,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package='m20_warehouse_inspection',
                executable='m20_phase4_acceptance',
                name='m20_phase4_acceptance',
                output='screen',
                parameters=[
                    {
                        'mode': acceptance_mode,
                        'timeout_sec': 900.0,
                        'config_path': system_config,
                    }
                ],
                condition=IfCondition(run_acceptance),
            ),
        ]
    )
