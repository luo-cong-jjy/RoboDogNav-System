"""Start only the M20 MuJoCo factory backend and its isolated viewer."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav = FindPackageShare('m20_nav2_system')
    backend_pkg = FindPackageShare('m20_nav2_backend')
    backend = PathJoinSubstitution([
        backend_pkg, 'launch',
        'mujoco_backend.launch.py',
    ])
    return LaunchDescription([
        Node(
            package='m20_nav2_system',
            executable='m20_factory_scene_markers',
            name='m20_factory_scene_markers',
            output='screen',
            parameters=[{
                'world_file': PathJoinSubstitution([
                    nav, 'worlds', 'factory_environment_mujoco.world',
                ]),
                'frame_id': 'map',
                'dynamic_speed_scale': 0.35,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(backend),
            launch_arguments={
                'system_config': PathJoinSubstitution([
                    backend_pkg, 'config', 'factory_backend.yaml',
                ]),
                'world_source': 'factory_sdf',
                'world_file': PathJoinSubstitution([
                    nav, 'worlds', 'factory_environment_mujoco.world',
                ]),
                'sdf_extractor': PathJoinSubstitution([
                    nav, '..', '..', 'lib', 'm20_nav2_system',
                    'gazebo_world_to_mujoco_geoms.py',
                ]),
                'mjcf_builder': PathJoinSubstitution([
                    nav, '..', '..', 'lib', 'm20_nav2_system',
                    'build_m20_mujoco_factory_world.py',
                ]),
                # Match m20_mujoco_navigation.launch.py and the Nav2 map origin.
                # Otherwise the backend inherits x=-37 from the warehouse config.
                'initial_x': '0.0',
                'initial_y': '0.0',
                'initial_yaw': '0.0',
                'use_viewer': 'true',
                'viewer_follow_camera': 'false',
                # Start almost vertically above the robot.  Avoid exactly
                # -90 degrees because azimuth is singular at the pole.
                'viewer_distance': '8.0',
                'viewer_azimuth': '90.0',
                'viewer_elevation': '-89.0',
                'viewer_max_fps': '30.0',
                'real_time_factor': '1.0',
                'parking_brake_enabled': 'false',
            }.items(),
        ),
    ])
