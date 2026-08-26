"""Start backend-neutral odometry and grid-LaserScan adapters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('body_pose_topic', default_value='/m20/sim/body_pose'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        Node(
            package='m20_nav2_system', executable='m20_mujoco_odom_adapter',
            name='m20_mujoco_odom_adapter', output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('body_pose_topic'),
                'output_topic': LaunchConfiguration('odom_topic'),
            }],
        ),
        Node(
            package='m20_nav2_system', executable='m20_grid_lidar_simulator',
            name='m20_grid_lidar_simulator', output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('m20_nav2_system'), 'config',
                'm20_grid_lidar_simulator.yaml']), {
                'map_topic': LaunchConfiguration('map_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'world_file': PathJoinSubstitution([
                    FindPackageShare('m20_nav2_system'), 'worlds',
                    'factory_environment.world']),
                'dynamic_speed_scale': 0.35,
            }],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='m20_base_to_scan_tf', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_scan'],
        ),
        # The simulated map and odometry share the same origin.  The odom
        # adapter publishes odom -> base_link; this closes the Nav2 TF tree.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='m20_map_to_odom_tf', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        Node(
            package='m20_nav2_system', executable='m20_factory_scene_markers',
            name='m20_factory_scene_markers', output='screen',
            parameters=[{'world_file': PathJoinSubstitution([
                FindPackageShare('m20_nav2_system'), 'worlds',
                'factory_environment.world']), 'dynamic_speed_scale': 0.35}],
        ),
    ])
