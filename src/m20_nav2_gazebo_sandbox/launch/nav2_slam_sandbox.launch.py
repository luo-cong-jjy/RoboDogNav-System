from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("m20_nav2_gazebo_sandbox"))
    nav2_params = str(pkg_share / "config" / "nav2_params.yaml")
    slam_params = str(pkg_share / "config" / "slam_toolbox.yaml")
    rviz_config = str(pkg_share / "rviz" / "nav2_sandbox.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_delay = LaunchConfiguration("rviz_delay")
    params_file = LaunchConfiguration("params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")

    nav2_bringup_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "bringup_launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_delay", default_value="8.0"),
        DeclareLaunchArgument("params_file", default_value=nav2_params),
        DeclareLaunchArgument("slam_params_file", default_value=slam_params),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                "slam": "True",
                "map": "",
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "slam_params_file": slam_params_file,
            }.items(),
        ),

        TimerAction(
            period=rviz_delay,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": use_sim_time}],
                    condition=IfCondition(use_rviz),
                ),
            ],
        ),
    ])
