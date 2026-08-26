"""Generate and validate the factory M20 MJCF asset."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('m20_nav2_system')
    sdk = FindPackageShare('m20_mujoco_backend')
    world = PathJoinSubstitution([pkg, 'worlds', 'factory_environment.world'])
    template = PathJoinSubstitution([sdk, 'models', 'm20_robot.xml'])
    meshdir = PathJoinSubstitution([FindPackageShare('m20_official_description'), 'meshes'])
    executable_dir = PathJoinSubstitution([pkg, '..', '..', 'lib', 'm20_nav2_system'])
    extractor = PathJoinSubstitution([executable_dir, 'gazebo_world_to_mujoco_geoms.py'])
    builder = PathJoinSubstitution([executable_dir, 'build_m20_mujoco_factory_world.py'])
    validator = PathJoinSubstitution([executable_dir, 'validate_m20_mujoco_factory_world.py'])
    geoms = LaunchConfiguration('geoms_output')
    output = LaunchConfiguration('mjcf_output')
    extract = ExecuteProcess(cmd=['python3', extractor, world, geoms], output='screen')
    build = ExecuteProcess(cmd=['python3', builder, template, geoms, output, '--meshdir', meshdir], output='screen')
    validate = ExecuteProcess(cmd=['python3', validator, output, '--expected-static-geoms', '10'], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('geoms_output', default_value='/tmp/m20_factory_geoms.xml'),
        DeclareLaunchArgument('mjcf_output', default_value='/tmp/m20_factory_m20.xml'),
        extract,
        RegisterEventHandler(OnProcessExit(target_action=extract, on_exit=[build])),
        RegisterEventHandler(OnProcessExit(target_action=build, on_exit=[validate])),
    ])
