"""Start the 2026-09-01 09:32 MuJoCo odometry and grid-LaserScan adapters."""

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
        DeclareLaunchArgument(
            'world_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('m20_nav2_system'), 'worlds',
                'factory_environment_mujoco.world',
            ]),
        ),
        # The authored trajectory is 1.0 m/s; the validated backend consumes
        # it at 0.35x, giving real workers a 0.35 m/s speed.
        DeclareLaunchArgument('dynamic_speed_scale', default_value='0.35'),
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
                'world_file': LaunchConfiguration('world_file'),
                'dynamic_speed_scale': LaunchConfiguration('dynamic_speed_scale'),
                # Exact runtime values from the 09:32-10:00 MuJoCo acceptance
                # session. /scan remains measured; /scan_predicted supplies
                # the early global-path detour signal.
                'dynamic_prediction_horizon': 7.0,
                'dynamic_prediction_max_horizon': 8.0,
                'dynamic_prediction_samples': 8,
                'dynamic_prediction_lead_time': 3.5,
                'dynamic_prediction_tail_time': 0.25,
                'robot_prediction_radius': 0.68,
                'dynamic_prediction_margin': 0.35,
                'prediction_base_distance': 2.5,
                'prediction_reaction_time': 7.0,
                'prediction_max_distance': 9.0,
            }],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='m20_base_to_scan_tf', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_scan'],
        ),
        # map -> odom is intentionally not published here.  The MuJoCo static
        # navigation launch is its sole owner; publishing it twice causes TF
        # races during Nav2 costmap initialization.
        Node(
            package='m20_nav2_system', executable='m20_factory_scene_markers',
            name='m20_factory_scene_markers', output='screen',
            parameters=[{
                'world_file': LaunchConfiguration('world_file'),
                'dynamic_speed_scale': LaunchConfiguration('dynamic_speed_scale'),
            }],
        ),
    ])
