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
from launch.conditions import IfCondition, UnlessCondition
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
    use_grid_route = LaunchConfiguration('use_grid_route')
    collision_grid_route_enabled = LaunchConfiguration(
        'collision_grid_route_enabled'
    )
    clearance_config = LaunchConfiguration('clearance_config')
    controller_config = LaunchConfiguration('controller_config')
    motion_backend = LaunchConfiguration('motion_backend')
    velocity_feedback_enabled = LaunchConfiguration(
        'velocity_feedback_enabled'
    )
    capability_profile = load_capability_profile(
        LaunchConfiguration(
            'locomotion_capability_config'
        ).perform(context)
    )
    intent_parameters = capability_profile.intent_parameters()
    controller_parameters = capability_profile.controller_parameters()
    guard_parameters = capability_profile.collision_guard_parameters()
    safety_parameters = capability_profile.safety_parameters()

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
                parameters=[{'cloud_topic': '/quad_0/cloud'}],
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
                        )
                    },
                ],
            ),
            Node(
                package='m20_inspection_core',
                executable='m20_safety_supervisor',
                output='screen',
                parameters=[
                    str(core / 'config' / 'safety_scan_vendor.yaml'),
                    safety_parameters,
                ],
            ),
            Node(
                package='m20_inspection_core',
                executable='m20_collision_guard',
                name='m20_collision_guard',
                output='screen',
                parameters=[
                    str(core / 'config' / 'collision_guard.yaml'),
                    clearance_config,
                    guard_parameters,
                ],
                condition=IfCondition(use_grid_route),
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
                ],
                condition=UnlessCondition(use_grid_route),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(scan / 'launch' / 'f1_scan.launch.py')
                ),
                launch_arguments={
                    # This is SCAN's native CPU ray-casting renderer. A small
                    # opt-in reload seam lets it rebuild after a floor switch.
                    'use_local_sensing': 'true',
                    'use_planner': use_planner,
                    'use_grid_route': use_grid_route,
                    'collision_grid_route_enabled': (
                        collision_grid_route_enabled
                    ),
                    'clearance_config': clearance_config,
                    'controller_config': controller_config,
                    'bidirectional_tracking_enabled': str(
                        controller_parameters[
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
                parameters=[{'config_path': str(system_config)}],
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
            DeclareLaunchArgument('use_grid_route', default_value='false'),
            DeclareLaunchArgument(
                'collision_grid_route_enabled',
                default_value='false',
                description=(
                    'Standby conservative grid route used only after the M20 '
                    'guard rejects a native SCAN trajectory.'
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
                    / 'clearance_vendor.yaml'
                ),
                description=(
                    'Vendor SCAN planning clearance. M20 execution safety '
                    'remains in the downstream collision guard.'
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
                'velocity_feedback_enabled',
                default_value='false',
                description=(
                    'Enable bounded measured body-velocity PI compensation '
                    'before collision prediction.'
                ),
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
