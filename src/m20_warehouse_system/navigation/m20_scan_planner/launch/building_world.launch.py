"""Launch Gazebo Classic with the 1_Building world model."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("m20_scan_planner"))
    models_dir = pkg_share / "models"
    building_model_root = models_dir / "1_Building"
    world_file = building_model_root / "Building.world"

    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    default_model_paths = [
        path for path in ("/usr/share/gazebo-11/models", "/usr/share/gazebo/models") if Path(path).is_dir()
    ]
    gazebo_model_path = os.pathsep.join(
        path
        for path in (
            str(building_model_root),
            *default_model_paths,
            existing_model_path,
        )
        if path
    )

    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")
    client_delay = LaunchConfiguration("client_delay")

    gzserver_cmd = [
        "gzserver",
        world_file.as_posix(),
        "-s",
        "libgazebo_ros_init.so",
        "-s",
        "libgazebo_ros_factory.so",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="true"),
            DeclareLaunchArgument("client_delay", default_value="1.0"),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            SetEnvironmentVariable("GAZEBO_MASTER_URI", "http://127.0.0.1:11345"),
            SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
            ExecuteProcess(
                cmd=gzserver_cmd + ["--verbose"],
                output="screen",
                condition=IfCondition(verbose),
            ),
            ExecuteProcess(
                cmd=gzserver_cmd,
                output="screen",
                condition=UnlessCondition(verbose),
            ),
            TimerAction(
                period=client_delay,
                actions=[
                    ExecuteProcess(
                        cmd=["gzclient"],
                        output="screen",
                        condition=IfCondition(gui),
                    )
                ],
            ),
        ]
    )
