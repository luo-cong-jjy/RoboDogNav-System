from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'config',
        'safety.yaml',
    ])

    return LaunchDescription([
        Node(
            package='m20_industrial_inspection',
            executable='cmd_vel_safety_mux_node',
            name='cmd_vel_safety_mux',
            output='screen',
            parameters=[config_file],
        ),
    ])
