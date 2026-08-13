from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="m20_industrial_inspection_gazebo",
            executable="keyboard_cmd_vel_teleop",
            name="keyboard_cmd_vel_teleop",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"cmd_topic": "/cmd_vel"},
                {"linear_step": 0.05},
                {"angular_step": 0.10},
                {"max_linear": 0.45},
                {"max_angular": 0.80},
                {"publish_rate_hz": 20.0},
            ],
        ),
    ])
