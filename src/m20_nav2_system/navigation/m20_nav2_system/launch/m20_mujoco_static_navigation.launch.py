"""Run map-based Nav2 for the MuJoCo motion-backend validation path.

This launch intentionally has no AMCL or slam_toolbox.  The MuJoCo backend,
the code-based LaserScan simulator, and factory_world_map.yaml all use the
same authored factory coordinates.  A fixed identity map->odom transform
preserves that coordinate contract while Nav2 supplies planning and control.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = Path(get_package_share_directory('m20_nav2_system'))
    params = str(share / 'config' / 'nav2_params.yaml')
    default_map = str(share / 'maps' / 'factory' / 'factory_world_map.yaml')
    rviz = str(share / 'rviz' / 'nav2_sandbox.rviz')
    navigation = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_delay', default_value='3.0'),
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'use_sim_time': False,
                'yaml_filename': LaunchConfiguration('map'),
            }],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map_server', output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
        # The backend publishes odom->base_link in authored factory coordinates.
        # No localization node may publish a competing map->odom transform here.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='m20_mujoco_map_to_odom', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        TimerAction(period=2.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'true',
                # nav2_bringup uses this value inside PythonExpression;
                # it must be Python's False, not the lowercase token false.
                'use_composition': 'False',
            }.items(),
        )]),
        TimerAction(period=LaunchConfiguration('rviz_delay'), actions=[Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz], parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        )]),
    ])
