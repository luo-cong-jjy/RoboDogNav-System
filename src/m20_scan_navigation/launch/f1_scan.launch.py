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


def generate_launch_description() -> LaunchDescription:
    """Create SCAN's canonical topic graph with two M20 boundary remaps."""
    share = Path(get_package_share_directory('m20_scan_navigation'))
    use_local_sensing = LaunchConfiguration('use_local_sensing')
    use_planner = LaunchConfiguration('use_planner')
    use_grid_route = LaunchConfiguration('use_grid_route')
    clearance_config = LaunchConfiguration('clearance_config')
    controller_config = LaunchConfiguration('controller_config')

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
        ],
        remappings=[
            ('body_pose', '/m20/sim/body_pose'),
            ('cmd_vel', '/m20/navigation/cmd_vel_raw'),
            ('heading_error', '/m20/navigation/heading_error'),
            ('heading_aligning', '/m20/navigation/heading_aligning'),
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
            {'scan_goal_topic': '/move_base_simple/goal'},
        ],
        condition=IfCondition(use_grid_route),
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
                'direct_goal_topic': '/move_base_simple/goal',
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
            local_sensing,
            route_planner,
            navigation_gateway,
            planner,
            controller,
        ]
    )
