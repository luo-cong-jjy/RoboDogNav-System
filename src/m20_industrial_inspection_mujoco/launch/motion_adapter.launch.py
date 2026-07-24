from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection_mujoco'),
        'config',
        'sim.yaml',
    ])

    return LaunchDescription([
        Node(
            package='m20_industrial_inspection_mujoco',
            executable='motion_adapter_node',
            name='motion_adapter_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
