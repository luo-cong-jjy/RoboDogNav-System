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

"""M20 warehouse inspection wired to the factory real-robot motion layer."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose Elevator-LIO, navigation, and guarded factory locomotion."""
    integration = Path(
        get_package_share_directory('m20_warehouse_inspection')
    )
    locomotion = Path(
        get_package_share_directory('m20_locomotion_control')
    )
    default_system = (
        integration / 'config' / 'sites' / 'm20_pao_f1_template.yaml'
    )
    if not default_system.is_file():
        raise RuntimeError(
            'hardware site profile is not installed; pass system_config '
            'only after installing a surveyed-site configuration'
        )
    default_capability = (
        locomotion
        / 'config'
        / 'm20_factory_agile_flat_capabilities.yaml'
    )
    system_config = LaunchConfiguration('system_config')
    capability = LaunchConfiguration('locomotion_capability_config')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'system_config', default_value=str(default_system)
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_planner', default_value='true'),
            DeclareLaunchArgument(
                'start_elevator_lio',
                default_value='true',
                description=(
                    'Start the validated Elevator-LIO package in this '
                    'bring-up. Set false when LIO is managed separately.'
                ),
            ),
            DeclareLaunchArgument(
                'lio_config_path',
                default_value='root_config_m20_navigation_relocation.yaml',
                description=(
                    'Elevator-LIO root YAML filename under lio/yaml. The '
                    'hardware default loads the commissioned F1 map and '
                    'publishes world -> base_link; mapping is a separate '
                    'commissioning command.'
                ),
            ),
            DeclareLaunchArgument(
                'start_localization_adapter',
                default_value='true',
                description=(
                    'Fuse Elevator-LIO body pose with factory measured '
                    'Twist into the canonical navigation Odometry.'
                ),
            ),
            DeclareLaunchArgument(
                'lio_body_pose_topic',
                default_value='/LIO/odom_vehicle',
            ),
            DeclareLaunchArgument(
                'body_pose_topic',
                default_value='/m20/localization/body_pose',
                description=(
                    'Real localization Odometry in the active map frame.'
                ),
            ),
            DeclareLaunchArgument(
                'navigation_cloud_topic',
                default_value='/LIO/clouds_lidar',
                description=(
                    'Elevator-LIO deskewed live cloud transformed into the '
                    'SCAN world frame; raw lidar points are not sufficient.'
                ),
            ),
            DeclareLaunchArgument(
                'sensor_pose_topic',
                default_value='/LIO/odom_imu',
            ),
            DeclareLaunchArgument(
                'relocation_service',
                default_value='/m20/localization/set_pose',
                description=(
                    'Per-floor localization reinitialization service. It '
                    'must implement '
                    'm20_warehouse_interfaces/SetSimulationPose; the '
                    'simulation-oriented type name is retained only for '
                    'wire compatibility.'
                ),
            ),
            DeclareLaunchArgument(
                'locomotion_capability_config',
                default_value=str(default_capability),
            ),
            DeclareLaunchArgument(
                'robot_host', default_value='10.21.31.103'
            ),
            DeclareLaunchArgument(
                'factory_transport', default_value='basic_server'
            ),
            DeclareLaunchArgument(
                'command_ownership_confirmed', default_value='false'
            ),
            DeclareLaunchArgument(
                'auto_enable_motion', default_value='false'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('lio'),
                            'launch',
                            'start_ros2.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'config_path': LaunchConfiguration('lio_config_path'),
                    # The integrated RViz already contains SCAN and robot
                    # displays; do not create a second RViz process.
                    'use_rviz': 'false',
                }.items(),
                condition=IfCondition(
                    LaunchConfiguration('start_elevator_lio')
                ),
            ),
            Node(
                package='m20_warehouse_inspection',
                executable='m20_hardware_localization_adapter',
                name='m20_hardware_localization_adapter',
                output='screen',
                parameters=[
                    {
                        'input_odometry_topic': LaunchConfiguration(
                            'lio_body_pose_topic'
                        ),
                        'output_odometry_topic': LaunchConfiguration(
                            'body_pose_topic'
                        ),
                        'measured_twist_topic': (
                            '/m20/locomotion/measured_twist'
                        ),
                        'expected_world_frame': 'world',
                        'expected_body_frame': 'base_link',
                    }
                ],
                condition=IfCondition(
                    LaunchConfiguration('start_localization_adapter')
                ),
            ),
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
                    'use_rviz': LaunchConfiguration('use_rviz'),
                    'use_planner': LaunchConfiguration('use_planner'),
                    'use_local_sensing': 'false',
                    'motion_backend': 'external',
                    # Real hardware remains fail-closed until a dedicated
                    # native-SCAN/M20 physical acceptance has been completed.
                    'execution_profile': 'm20_safe',
                    'velocity_feedback_enabled': 'true',
                    'velocity_feedback_source': 'twist',
                    'velocity_feedback_twist_topic': (
                        '/m20/locomotion/measured_twist'
                    ),
                    'body_pose_topic': LaunchConfiguration(
                        'body_pose_topic'
                    ),
                    'navigation_cloud_topic': LaunchConfiguration(
                        'navigation_cloud_topic'
                    ),
                    'sensor_pose_topic': LaunchConfiguration(
                        'sensor_pose_topic'
                    ),
                    'relocation_service': LaunchConfiguration(
                        'relocation_service'
                    ),
                    'locomotion_capability_config': capability,
                    'navigation_timeout_sec': '300.0',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        locomotion
                        / 'launch'
                        / 'official_locomotion.launch.py'
                    )
                ),
                launch_arguments={
                    'factory_transport': LaunchConfiguration(
                        'factory_transport'
                    ),
                    'locomotion_capability_config': capability,
                    'robot_host': LaunchConfiguration('robot_host'),
                    'command_ownership_confirmed': LaunchConfiguration(
                        'command_ownership_confirmed'
                    ),
                    'auto_enable_motion': LaunchConfiguration(
                        'auto_enable_motion'
                    ),
                }.items(),
            ),
        ]
    )
