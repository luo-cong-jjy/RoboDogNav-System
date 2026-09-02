"""Run current 2-D Nav2 with the official M20 MuJoCo backend."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    nav = FindPackageShare('m20_nav2_system')
    backend_pkg = FindPackageShare('m20_nav2_backend')
    backend = PathJoinSubstitution([backend_pkg, 'launch', 'mujoco_backend.launch.py'])
    locomotion = PathJoinSubstitution([FindPackageShare('m20_nav2_locomotion'), 'launch', 'sdk_locomotion.launch.py'])
    bridge = PathJoinSubstitution([nav, 'launch', 'm20_mujoco_sensor_bridge.launch.py'])
    # This is deliberately not nav2_map_navigation_m20_factory.launch.py:
    # that launch includes AMCL for saved-map Gazebo/hardware navigation.
    # MuJoCo shares factory coordinates with the grid-lidar simulator and
    # therefore needs a fixed map->odom identity transform instead.
    nav2 = PathJoinSubstitution([nav, 'launch', 'm20_mujoco_static_navigation.launch.py'])
    libexec = PathJoinSubstitution([nav, '..', '..', 'lib', 'm20_nav2_system'])
    return LaunchDescription([
        DeclareLaunchArgument('system_config', default_value=PathJoinSubstitution([backend_pkg, 'config', 'factory_backend.yaml'])),
        # Preserve the 2026-09-01 09:32 validated MuJoCo scene independently
        # from Gazebo's mapping and saved-map navigation worlds.
        DeclareLaunchArgument('world_file', default_value=PathJoinSubstitution([nav, 'worlds', 'factory_environment_mujoco.world'])),
        # MuJoCo uses the authored static simulation map.  This is deliberately
        # separate from m20_factory_slam.yaml, which is an exported SLAM map
        # used by the Gazebo/hardware saved-map workflows.
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([nav, 'maps', 'factory', 'factory_world_map.yaml'])),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # Nav2/map_server starts after nav2_delay; open RViz after its map is
        # active so the initial map subscription is deterministic in WSL2.
        DeclareLaunchArgument('rviz_delay', default_value='8.0'),
        # The isolated GLFW viewer is optional diagnostics.  Keep it off by
        # default because WSLg/OpenGL may expose a blank framebuffer; Nav2
        # and the physics backend remain fully usable without it.
        DeclareLaunchArgument('use_mujoco_viewer', default_value='true'),
        # The physics backend needs a few seconds to generate/load the MJCF
        # and publish a valid joint state before the SDK consumes it.
        DeclareLaunchArgument('sdk_delay', default_value='2.0'),
        # Wait for the RL stand-up/backend clock and odom TF before Nav2
        # lifecycle activation; otherwise costmaps query a pre-TF timestamp.
        DeclareLaunchArgument('nav2_delay', default_value='10.0'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        LogInfo(msg=[
            '[m20_nav2_system] MuJoCo 09:32 baseline factory world: ',
            LaunchConfiguration('world_file'),
        ]),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='m20_robot_state_publisher', output='screen',
            parameters=[{
                'robot_description': (
                    Path(get_package_share_directory('m20_nav2_system')) /
                    'models' / 'urdf' / 'M20_official_visual.urdf'
                ).read_text(encoding='utf-8'),
                'use_sim_time': False,
            }],
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(backend), launch_arguments={
            'system_config': LaunchConfiguration('system_config'), 'world_source': 'factory_sdf',
            'world_file': LaunchConfiguration('world_file'),
            'sdf_extractor': PathJoinSubstitution([libexec, 'gazebo_world_to_mujoco_geoms.py']),
            'mjcf_builder': PathJoinSubstitution([libexec, 'build_m20_mujoco_factory_world.py']),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            # Match the standalone viewer's wider initial factory view.
            'viewer_distance': '8.0',
            'viewer_azimuth': '90.0',
            'viewer_elevation': '-89.0',
            'viewer_follow_camera': 'false',
            # The official RL policy owns wheel commands in this validation
            # path.  Do not let a project-side text mode brake its output.
            'parking_brake_enabled': 'false',
            'use_viewer': LaunchConfiguration('use_mujoco_viewer')}.items()),
        TimerAction(period=LaunchConfiguration('sdk_delay'), actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(locomotion), launch_arguments={
                'start_sdk': 'true', 'require_backend_ready': 'true', 'input_topic': '/cmd_vel',
                'execution_profile': 'scan_native',
                # Keep Nav2's velocity-smoother as the sole final command
                # source. Dynamic prediction is supplied through
                # /scan_predicted to the global costmap, rather than
                # overriding the final control stream.
                'sdk_cmd_vel_topic': '/cmd_vel',
                'sdk_debug_print': 'true'}.items()),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge),
            launch_arguments={
                'world_file': LaunchConfiguration('world_file'),
                'dynamic_speed_scale': '0.35',
            }.items(),
        ),
        # RViz publishes PoseStamped while Nav2 accepts NavigateToPose actions.
        # Keep this conversion explicit so the MuJoCo backend only ever sees
        # the final /cmd_vel output from Nav2.
        Node(
            package='m20_nav2_system', executable='m20_nav2_goal_bridge',
            name='m20_nav2_goal_bridge', output='screen',
            parameters=[{'input_topic': '/m20_nav2/goal_pose_raw'}],
        ),
        TimerAction(period=LaunchConfiguration('nav2_delay'), actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2), launch_arguments={
                'map': LaunchConfiguration('map'), 'use_rviz': LaunchConfiguration('use_rviz'),
                'rviz_delay': LaunchConfiguration('rviz_delay'),
                # MuJoCo backend currently publishes wall-clock timestamps and
                # does not provide /clock; keep Nav2 on the same clock.
                'use_sim_time': 'false'}.items())]),
    ])
