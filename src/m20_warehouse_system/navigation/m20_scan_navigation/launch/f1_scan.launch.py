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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create SCAN's canonical topic graph with two M20 boundary remaps."""
    share = Path(get_package_share_directory('m20_scan_navigation'))
    use_local_sensing = LaunchConfiguration('use_local_sensing')
    use_planner = LaunchConfiguration('use_planner')
    body_pose_topic = LaunchConfiguration('body_pose_topic')
    navigation_cloud_topic = LaunchConfiguration(
        'navigation_cloud_topic'
    )
    sensor_pose_topic = LaunchConfiguration('sensor_pose_topic')
    clearance_config = LaunchConfiguration('clearance_config')
    controller_config = LaunchConfiguration('controller_config')
    require_external_execution_hold = LaunchConfiguration(
        'require_external_execution_hold'
    )
    collision_guard_required = LaunchConfiguration(
        'collision_guard_required'
    )
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
            str(share / 'config' / 'scan_vendor_local_sensing.yaml'),
            {'body_pose_topic': body_pose_topic},
        ],
        remappings=[
            ('global_map', '/map_generator/global_cloud'),
            ('cloud', navigation_cloud_topic),
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
            ('body_pose', body_pose_topic),
            ('sensor_pose', sensor_pose_topic),
            ('cloud', navigation_cloud_topic),
            ('reset', '/m20/navigation/reset'),
            ('move_base_simple/goal', '/move_base_simple/goal'),
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
                'require_external_execution_hold': ParameterValue(
                    require_external_execution_hold,
                    value_type=bool,
                ),
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
            ('body_pose', body_pose_topic),
            ('cmd_vel', '/m20/navigation/cmd_vel_raw'),
        ],
        condition=IfCondition(use_planner),
    )
    navigation_gateway = Node(
        package='m20_scan_navigation',
        executable='m20_navigation_gateway',
        name='m20_navigation_gateway',
        output='screen',
        parameters=[
            str(share / 'config' / 'navigation_gateway.yaml'),
            {
                'manual_goal_monitor_enabled': False,
                'direct_goal_topic': '/move_base_simple/goal',
                'odom_topic': body_pose_topic,
                'collision_guard_required': ParameterValue(
                    collision_guard_required,
                    value_type=bool,
                ),
            },
        ],
        condition=IfCondition(use_planner),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_local_sensing', default_value='true'),
            DeclareLaunchArgument(
                'body_pose_topic',
                default_value='/m20/sim/body_pose',
                description=(
                    'Odometry source shared by SCAN, safety, and hardware '
                    'localization adapters.'
                ),
            ),
            DeclareLaunchArgument(
                'navigation_cloud_topic',
                default_value='/quad_0/cloud',
                description='Map/world-frame cloud consumed by SCAN.',
            ),
            DeclareLaunchArgument(
                'sensor_pose_topic',
                default_value='/quad_0/lidar_pose',
                description='Odometry-form sensor pose consumed by SCAN.',
            ),
            DeclareLaunchArgument('use_planner', default_value='true'),
            DeclareLaunchArgument(
                'clearance_config',
                default_value=str(
                    share / 'config' / 'clearance_vendor.yaml'
                ),
                description=(
                    'ROS parameter override for native SCAN and the '
                    'independent online-point-cloud collision guard.'
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
                'require_external_execution_hold',
                default_value='false',
                description=(
                    'Optional M20 safety integration. Native SCAN advances '
                    'its trajectory clock exactly as upstream.'
                ),
            ),
            DeclareLaunchArgument(
                'collision_guard_required',
                default_value='true',
                description=(
                    'Require the independent online-cloud guard before '
                    'typed/managed goals are relayed to native SCAN.'
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
            navigation_gateway,
            planner,
            controller,
        ]
    )
