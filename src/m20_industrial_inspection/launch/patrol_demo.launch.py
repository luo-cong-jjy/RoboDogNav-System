from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    safety_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'config',
        'safety.yaml',
    ])
    sim_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'config',
        'sim.yaml',
    ])
    patrol_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'config',
        'patrol_routes.yaml',
    ])

    return LaunchDescription([
        Node(
            package='m20_industrial_inspection',
            executable='cmd_vel_safety_mux_node',
            name='cmd_vel_safety_mux',
            output='screen',
            parameters=[safety_config],
        ),
        Node(
            package='m20_industrial_inspection',
            executable='motion_adapter_node',
            name='motion_adapter_node',
            output='screen',
            parameters=[sim_config],
        ),
        Node(
            package='m20_industrial_inspection',
            executable='patrol_manager_node',
            name='patrol_manager_node',
            output='screen',
            parameters=[patrol_config],
        ),
    ])
