from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='m20_hardware_preflight',
            executable='m20_hardware_preflight',
            name='m20_hardware_preflight',
            output='screen',
        ),
    ])
