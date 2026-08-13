from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("m20_nav2_gazebo_sandbox"),
        "launch",
        "gazebo_sensor_m20_factory.launch.py",
    ])

    nav2_launch = PathJoinSubstitution([
        FindPackageShare("m20_nav2_gazebo_sandbox"),
        "launch",
        "nav2_map_navigation_m20_factory.launch.py",
    ])

    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    rviz_delay = LaunchConfiguration("rviz_delay")
    nav2_delay = LaunchConfiguration("nav2_delay")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gazebo_gui", default_value="false"),
        DeclareLaunchArgument("rviz_delay", default_value="10.0"),
        DeclareLaunchArgument("nav2_delay", default_value="5.0"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.59"),
        DeclareLaunchArgument("yaw", default_value="0.0"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "use_rviz": "false",
                "use_gazebo_gui": use_gazebo_gui,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
            }.items(),
        ),

        TimerAction(
            period=nav2_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        "use_rviz": use_rviz,
                        "rviz_delay": rviz_delay,
                    }.items(),
                ),
            ],
        ),
    ])
