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

"""Launch phase 3: bidirectional flat multi-floor SCAN simulation."""

from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from m20_locomotion_control.capability_profile import (
    load_capability_profile,
)


def _runtime_actions(context):
    """Build nodes after resolving the selected system configuration."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    description = Path(
        get_package_share_directory('m20_official_description')
    )
    scan = Path(get_package_share_directory('m20_scan_navigation'))
    simulation = Path(get_package_share_directory('m20_warehouse_sim'))
    core = Path(get_package_share_directory('m20_inspection_core'))
    locomotion = Path(
        get_package_share_directory('m20_locomotion_control')
    )
    scan_vendor = Path(get_package_share_directory('m20_scan_planner'))

    system_config = Path(
        LaunchConfiguration('system_config').perform(context)
    ).expanduser().resolve()
    with system_config.open('r', encoding='utf-8') as stream:
        initial_pose = yaml.safe_load(stream)['floors']['F1']['initial_pose']
    model = description / 'urdf' / 'm20_official.urdf'
    # Load the exact RViz profile installed by the vendored SCAN package.
    rviz = scan_vendor / 'rviz' / 'default.rviz'
    use_rviz = LaunchConfiguration('use_rviz')
    use_planner = LaunchConfiguration('use_planner')
    use_local_sensing = LaunchConfiguration('use_local_sensing')
    clearance_config = LaunchConfiguration('clearance_config')
    controller_config = LaunchConfiguration('controller_config')
    motion_backend = LaunchConfiguration('motion_backend')
    execution_profile = LaunchConfiguration(
        'execution_profile'
    ).perform(context)
    if execution_profile not in {'scan_native', 'm20_safe'}:
        raise RuntimeError(
            'execution_profile must be scan_native or m20_safe'
        )
    scan_native = execution_profile == 'scan_native'
    velocity_feedback_enabled = LaunchConfiguration(
        'velocity_feedback_enabled'
    )
    velocity_feedback_source = LaunchConfiguration(
        'velocity_feedback_source'
    )
    velocity_feedback_twist_topic = LaunchConfiguration(
        'velocity_feedback_twist_topic'
    )
    body_pose_topic = LaunchConfiguration('body_pose_topic')
    navigation_cloud_topic = LaunchConfiguration(
        'navigation_cloud_topic'
    )
    sensor_pose_topic = LaunchConfiguration('sensor_pose_topic')
    relocation_service = LaunchConfiguration('relocation_service')
    capability_profile = load_capability_profile(
        LaunchConfiguration(
            'locomotion_capability_config'
        ).perform(context)
    )
    intent_parameters = capability_profile.intent_parameters()
    controller_parameters = capability_profile.controller_parameters()
    guard_parameters = capability_profile.collision_guard_parameters()
    safety_parameters = capability_profile.safety_parameters()
    native_safety_parameters = (
        {
            # Preserve SCAN's upstream holonomic Twist and numeric envelope.
            # This supervisor is only the indispensable multi-floor/e-stop
            # gate in the native profile; it performs no obstacle veto.
            'navigation_topic': '/m20/navigation/cmd_vel_raw',
            'collision_guard_enabled': False,
            'max_linear_x': 0.75,
            'max_linear_y': 0.35,
            'max_angular_z': 1.0,
            'max_linear_accel': 50.0,
            'max_angular_accel': 50.0,
        }
        if scan_native
        else {}
    )

    return [
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
                        'republish_period_sec': 0.0,
                        'switch_commit_delay_sec': 0.05,
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
                        # Keep the visible robot byte-for-byte equivalent in
                        # structure to the official M20 URDF.
                        'robot_description': model.read_text(encoding='utf-8')
                    }
                ],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='m20_map_to_scan_world',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--yaw', '0', '--pitch', '0', '--roll', '0',
                    '--frame-id', 'map', '--child-frame-id', 'world',
                ],
                output='screen',
            ),
            Node(
                package='m20_warehouse_sim',
                executable='m20_rviz_kinematic_backend',
                output='screen',
                parameters=[
                    str(simulation / 'config' / 'rviz_scan_vendor.yaml'),
                    {
                        # The system configuration is the single source of
                        # truth when map offsets or initial poses change.
                        'initial_x': float(initial_pose[0]),
                        'initial_y': float(initial_pose[1]),
                        'initial_yaw': float(initial_pose[2]),
                    },
                ],
                condition=IfCondition(
                    PythonExpression(
                        ["'", motion_backend, "' == 'rviz'"]
                    )
                ),
            ),
            Node(
                package='m20_multifloor_map',
                executable='m20_vendor_sensing_state',
                output='screen',
                parameters=[{'cloud_topic': navigation_cloud_topic}],
            ),
            Node(
                package='m20_locomotion_control',
                executable='m20_navigation_adapter',
                name='m20_navigation_adapter',
                output='screen',
                parameters=[
                    str(locomotion / 'config' / 'sdk_locomotion.yaml'),
                    intent_parameters,
                    {
                        'velocity_feedback_enabled': ParameterValue(
                            velocity_feedback_enabled,
                            value_type=bool,
                        ),
                        'velocity_feedback_odometry_topic': body_pose_topic,
                        'velocity_feedback_source': (
                            velocity_feedback_source
                        ),
                        'velocity_feedback_twist_topic': (
                            velocity_feedback_twist_topic
                        ),
                    },
                ],
                condition=IfCondition(str(not scan_native).lower()),
            ),
            Node(
                package='m20_inspection_core',
                executable='m20_safety_supervisor',
                output='screen',
                parameters=[
                    str(core / 'config' / 'safety_scan_vendor.yaml'),
                    safety_parameters,
                    native_safety_parameters,
                    {'odom_topic': body_pose_topic},
                ],
            ),
            Node(
                package='m20_inspection_core',
                executable='m20_collision_guard',
                name='m20_collision_guard',
                output='screen',
                parameters=[
                    str(core / 'config' / 'collision_guard_scan_native.yaml'),
                    clearance_config,
                    guard_parameters,
                    {'odom_topic': body_pose_topic},
                ],
                condition=IfCondition(str(not scan_native).lower()),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(scan / 'launch' / 'f1_scan.launch.py')
                ),
                launch_arguments={
                    # This is SCAN's native CPU ray-casting renderer. A small
                    # opt-in reload seam lets it rebuild after a floor switch.
                    'use_local_sensing': use_local_sensing,
                    'use_planner': use_planner,
                    'body_pose_topic': body_pose_topic,
                    'navigation_cloud_topic': navigation_cloud_topic,
                    'sensor_pose_topic': sensor_pose_topic,
                    'clearance_config': clearance_config,
                    'controller_config': controller_config,
                    'require_external_execution_hold': str(
                        not scan_native
                    ).lower(),
                    'collision_guard_required': str(
                        not scan_native
                    ).lower(),
                    'bidirectional_tracking_enabled': str(
                        False
                        if scan_native
                        else controller_parameters[
                            'bidirectional_tracking_enabled'
                        ]
                    ).lower(),
                    'reverse_tracking_enter_angle': str(
                        controller_parameters[
                            'reverse_tracking_enter_angle'
                        ]
                    ),
                    'reverse_tracking_exit_angle': str(
                        controller_parameters[
                            'reverse_tracking_exit_angle'
                        ]
                    ),
                    'reverse_tracking_min_hold_sec': str(
                        controller_parameters[
                            'reverse_tracking_min_hold_sec'
                        ]
                    ),
                    'reverse_tracking_entry_alignment': str(
                        controller_parameters[
                            'reverse_tracking_entry_alignment'
                        ]
                    ),
                    'reverse_tracking_exit_alignment': str(
                        controller_parameters[
                            'reverse_tracking_exit_alignment'
                        ]
                    ),
                }.items(),
            ),
            Node(
                package='m20_inspection_core',
                executable='m20_floor_switch_manager',
                output='screen',
                parameters=[
                    {
                        'config_path': str(system_config),
                        'body_pose_topic': body_pose_topic,
                        'set_pose_service': relocation_service,
                    }
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='m20_phase3_multifloor_rviz',
                arguments=['-d', str(rviz)],
                output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]


def generate_launch_description() -> LaunchDescription:
    """Connect the selected profile to the atomic floor-switch graph."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    default_config = (
        integration / 'config' / 'flat_multifloor_system.yaml'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'system_config',
                default_value=str(default_config),
                description='Absolute system profile YAML path.',
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_planner', default_value='true'),
            DeclareLaunchArgument(
                'use_local_sensing',
                default_value='true',
                description=(
                    'Use SCAN PCD ray casting in simulation; hardware '
                    'launches disable it and provide a live cloud.'
                ),
            ),
            DeclareLaunchArgument(
                'body_pose_topic',
                default_value='/m20/sim/body_pose',
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
                description=(
                    'Simulation teleport service or a real localization '
                    'reinitialization service with the same contract.'
                ),
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
                    'M20 body-geometry override layered on the otherwise '
                    'native SCAN planner. Use clearance_vendor.yaml for an '
                    'exact upstream geometry A/B run.'
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
                    / 'scan_vendor_controller.yaml'
                ),
                description=(
                    'Closed-loop controller profile. The complete system '
                    'defaults to the vendor SCAN values.'
                ),
            ),
            DeclareLaunchArgument(
                'motion_backend',
                default_value='rviz',
                description=(
                    'rviz starts the planar backend; external reserves the '
                    'pose, joint-state and TF contract for MuJoCo or hardware.'
                ),
            ),
            DeclareLaunchArgument(
                'execution_profile',
                default_value='scan_native',
                description=(
                    'scan_native preserves the upstream SCAN command path; '
                    'm20_safe enables the experimental rolling adapter, '
                    'footprint guard and trajectory hold chain.'
                ),
            ),
            DeclareLaunchArgument(
                'velocity_feedback_enabled',
                default_value='false',
                description=(
                    'Enable bounded measured body-velocity PI compensation '
                    'before collision prediction.'
                ),
            ),
            DeclareLaunchArgument(
                'velocity_feedback_source',
                default_value='odometry',
                description='Measured velocity source: odometry or twist.',
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
                description=(
                    'Versioned M20 command, turn and recovery capability '
                    'profile shared by motion adaptation and collision guard.'
                ),
            ),
            OpaqueFunction(function=_runtime_actions),
        ]
    )
