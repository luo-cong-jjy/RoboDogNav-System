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

"""Launch the vendored SCAN first-scene graph around the M20 interfaces."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create SCAN's canonical topic graph with two M20 boundary remaps."""
    share = Path(get_package_share_directory('m20_scan_navigation'))
    use_local_sensing = LaunchConfiguration('use_local_sensing')
    use_planner = LaunchConfiguration('use_planner')
    use_grid_route = LaunchConfiguration('use_grid_route')
    collision_grid_route_enabled = LaunchConfiguration(
        'collision_grid_route_enabled'
    )
    managed_scan_goal_topic = PythonExpression(
        [
            "'/m20/navigation/scan_goal_internal' if '",
            collision_grid_route_enabled,
            "' == 'true' else '/move_base_simple/goal'",
        ]
    )
    clearance_config = LaunchConfiguration('clearance_config')
    controller_config = LaunchConfiguration('controller_config')
    bidirectional_tracking_enabled = LaunchConfiguration(
        'bidirectional_tracking_enabled'
    )
    reverse_tracking_enter_angle = LaunchConfiguration(
        'reverse_tracking_enter_angle'
    )
    reverse_tracking_exit_angle = LaunchConfiguration(
        'reverse_tracking_exit_angle'
    )
    reverse_tracking_min_hold_sec = LaunchConfiguration(
        'reverse_tracking_min_hold_sec'
    )
    reverse_tracking_entry_alignment = LaunchConfiguration(
        'reverse_tracking_entry_alignment'
    )
    reverse_tracking_exit_alignment = LaunchConfiguration(
        'reverse_tracking_exit_alignment'
    )

    local_sensing = Node(
        package='local_sensing_node',
        executable='pcl_render_node',
        name='pcl_render_node',
        output='screen',
        parameters=[
            str(share / 'config' / 'scan_vendor_local_sensing.yaml')
        ],
        remappings=[
            ('global_map', '/map_generator/global_cloud'),
            ('cloud', '/quad_0/cloud'),
            ('sensor_cloud', '/quad_0/sensor_cloud'),
            ('dyn_cloud', '/quad_0/dyn_cloud'),
            ('uav_cloud', '/quad_0/uav_cloud'),
        ],
        condition=IfCondition(use_local_sensing),
    )
    planner = Node(
        # The algorithm implementation is the direct SCAN source mirror.
        # Only body odometry and reset cross the M20 integration boundary.
        package='m20_scan_planner',
        executable='scan_planner_node',
        name='scan_planner_node',
        output='screen',
        parameters=[
            str(share / 'config' / 'scan_vendor_planner.yaml'),
            clearance_config,
        ],
        remappings=[
            ('body_pose', '/m20/sim/body_pose'),
            ('sensor_pose', '/quad_0/lidar_pose'),
            ('cloud', '/quad_0/cloud'),
            ('reset', '/m20/navigation/reset'),
            ('move_base_simple/goal', managed_scan_goal_topic),
        ],
        condition=IfCondition(use_planner),
    )
    controller = Node(
        package='m20_scan_planner',
        executable='closed_loop_controller',
        name='closed_loop_controller',
        output='screen',
        parameters=[
            str(share / 'config' / 'scan_vendor_controller.yaml'),
            controller_config,
            {
                'bidirectional_tracking_enabled': ParameterValue(
                    bidirectional_tracking_enabled,
                    value_type=bool,
                ),
                'reverse_tracking_enter_angle': ParameterValue(
                    reverse_tracking_enter_angle,
                    value_type=float,
                ),
                'reverse_tracking_exit_angle': ParameterValue(
                    reverse_tracking_exit_angle,
                    value_type=float,
                ),
                'reverse_tracking_min_hold_sec': ParameterValue(
                    reverse_tracking_min_hold_sec,
                    value_type=float,
                ),
                'reverse_tracking_entry_alignment': ParameterValue(
                    reverse_tracking_entry_alignment,
                    value_type=float,
                ),
                'reverse_tracking_exit_alignment': ParameterValue(
                    reverse_tracking_exit_alignment,
                    value_type=float,
                ),
            },
        ],
        remappings=[
            ('body_pose', '/m20/sim/body_pose'),
            ('cmd_vel', '/m20/navigation/cmd_vel_raw'),
        ],
        condition=IfCondition(use_planner),
    )
    route_planner = Node(
        package='m20_scan_navigation',
        executable='m20_grid_route_planner',
        name='m20_grid_route_planner',
        output='screen',
        parameters=[
            str(share / 'config' / 'f1_grid_route.yaml'),
            clearance_config,
            {
                'scan_goal_topic': ParameterValue(
                    managed_scan_goal_topic, value_type=str
                )
            },
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_grid_route,
                    "' == 'true' or '",
                    collision_grid_route_enabled,
                    "' == 'true'",
                ]
            )
        ),
    )
    navigation_gateway = Node(
        package='m20_scan_navigation',
        executable='m20_navigation_gateway',
        name='m20_navigation_gateway',
        output='screen',
        parameters=[
            str(share / 'config' / 'navigation_gateway.yaml'),
            {
                'use_grid_route': use_grid_route,
                'collision_grid_route_enabled': ParameterValue(
                    collision_grid_route_enabled,
                    value_type=bool,
                ),
                'manual_goal_monitor_enabled': ParameterValue(
                    collision_grid_route_enabled,
                    value_type=bool,
                ),
                'manual_goal_topic': '/move_base_simple/goal',
                'direct_goal_topic': ParameterValue(
                    managed_scan_goal_topic, value_type=str
                ),
                'routed_goal_topic': '/m20/navigation/goal_pose',
            },
        ],
        condition=IfCondition(use_planner),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_local_sensing', default_value='true'),
            DeclareLaunchArgument('use_planner', default_value='true'),
            DeclareLaunchArgument(
                'use_grid_route',
                default_value='false',
                description=(
                    'Use the optional 2-D A* subgoal adapter. False keeps '
                    'the native SCAN goal/trajectory chain.'
                ),
            ),
            DeclareLaunchArgument(
                'collision_grid_route_enabled',
                default_value='false',
                description=(
                    'Keep the static-grid adapter in standby and use it only '
                    'after the M20 footprint guard rejects a native path.'
                ),
            ),
            DeclareLaunchArgument(
                'clearance_config',
                default_value=str(
                    share / 'config' / 'clearance_vendor.yaml'
                ),
                description=(
                    'ROS parameter override for SCAN, optional A*, and the '
                    'independent collision guard. The standalone first scene '
                    'defaults to the exact vendor behaviour.'
                ),
            ),
            DeclareLaunchArgument(
                'controller_config',
                default_value=str(
                    share / 'config' / 'scan_vendor_controller.yaml'
                ),
                description=(
                    'Optional controller override layered after the exact '
                    'vendor parameters. Standalone SCAN remains vendor-like.'
                ),
            ),
            DeclareLaunchArgument(
                'bidirectional_tracking_enabled',
                default_value='false',
                description=(
                    'Opt-in M20 execution adaptation; standalone SCAN keeps '
                    'the vendor forward-only controller behavior.'
                ),
            ),
            DeclareLaunchArgument(
                'reverse_tracking_enter_angle',
                default_value='2.10',
            ),
            DeclareLaunchArgument(
                'reverse_tracking_exit_angle',
                default_value='1.75',
            ),
            DeclareLaunchArgument(
                'reverse_tracking_min_hold_sec',
                default_value='0.80',
            ),
            DeclareLaunchArgument(
                'reverse_tracking_entry_alignment',
                default_value='0.20',
            ),
            DeclareLaunchArgument(
                'reverse_tracking_exit_alignment',
                default_value='0.35',
            ),
            local_sensing,
            route_planner,
            navigation_gateway,
            planner,
            controller,
        ]
    )
