from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("m20_industrial_inspection_gazebo"),
        "launch",
        "gazebo_sensor_m20_factory.launch.py",
    ])

    slam_launch = PathJoinSubstitution([
        FindPackageShare("m20_industrial_inspection_gazebo"),
        "launch",
        "nav2_slam_sandbox.launch.py",
    ])
    default_world = PathJoinSubstitution([
        FindPackageShare("m20_industrial_inspection_gazebo"),
        "worlds",
        "factory_environment_v2.world",
    ])
    use_rviz = LaunchConfiguration("use_rviz")
    use_teleop = LaunchConfiguration("use_teleop")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    gazebo_gui_delay = LaunchConfiguration("gazebo_gui_delay")
    spawn_delay = LaunchConfiguration("spawn_delay")
    slam_delay = LaunchConfiguration("slam_delay")
    rviz_delay = LaunchConfiguration("rviz_delay")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    world = LaunchConfiguration("world")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_teleop", default_value="false"),
        DeclareLaunchArgument("use_gazebo_gui", default_value="true"),
        DeclareLaunchArgument("gazebo_gui_delay", default_value="1.0"),
        DeclareLaunchArgument("spawn_delay", default_value="4.0"),
        DeclareLaunchArgument("slam_delay", default_value="8.0"),
        DeclareLaunchArgument("rviz_delay", default_value="5.0"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.59"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("world", default_value=default_world),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "use_rviz": use_rviz,
                "use_gazebo_gui": use_gazebo_gui,
                "gazebo_gui_delay": gazebo_gui_delay,
                "spawn_delay": spawn_delay,
                "rviz_delay": rviz_delay,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "world": world,
            }.items(),
        ),

        TimerAction(
            period=slam_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_launch),
                    launch_arguments={
                        "use_rviz": "false",
                    }.items(),
                ),
            ],
        ),

        Node(
            package="m20_industrial_inspection_gazebo",
            executable="keyboard_cmd_vel_teleop",
            name="keyboard_cmd_vel_teleop",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"cmd_topic": "/cmd_vel"},
                {"linear_step": 0.04},
                {"angular_step": 0.12},
                {"max_linear": 0.30},
                {"max_angular": 0.95},
                {"publish_rate_hz": 20.0},
            ],
            condition=IfCondition(use_teleop),
        ),
    ])
