"""Launch the synchronized single-floor mock map world and spawn M20."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_package_model_path(package_name: str, package_share: Path) -> str:
    model_path_root = Path("/tmp") / f"{package_name}_gazebo_model_path_{os.getpid()}"
    model_path_root.mkdir(parents=True, exist_ok=True)
    package_link = model_path_root / package_name
    if not package_link.exists():
        package_link.symlink_to(package_share, target_is_directory=True)
    return str(model_path_root)


def generate_launch_description():
    scan_share = Path(get_package_share_directory("m20_scan_planner"))
    gazebo_prefix = Path(get_package_prefix("m20_industrial_inspection_gazebo"))

    default_world_file = scan_share / "models" / "mockamap_single_floor" / "mockamap_single_floor.world"
    default_gazebo_robot_urdf = scan_share / "models" / "m20" / "m20_gazebo_legged_no_lidar.urdf"
    visual_robot_urdf = scan_share / "models" / "m20" / "urdf" / "M20_official_visual.urdf"

    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    default_model_paths = [
        path for path in ("/usr/share/gazebo-11/models", "/usr/share/gazebo/models") if Path(path).is_dir()
    ]
    scan_package_model_path = _make_package_model_path("m20_scan_planner", scan_share)
    gazebo_model_path = os.pathsep.join(
        path
        for path in (
            scan_package_model_path,
            *default_model_paths,
            existing_model_path,
        )
        if path
    )

    plugin_path = str(gazebo_prefix / "lib")
    existing_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    gazebo_plugin_path = (
        plugin_path
        if not existing_plugin_path
        else f"{plugin_path}{os.pathsep}{existing_plugin_path}"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")
    client_delay = LaunchConfiguration("client_delay")
    spawn_delay = LaunchConfiguration("spawn_delay")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    gazebo_robot_urdf = LaunchConfiguration("gazebo_robot_urdf")
    publish_standing_joints = LaunchConfiguration("publish_standing_joints")
    gazebo_master_uri = LaunchConfiguration("gazebo_master_uri")
    world_file = LaunchConfiguration("world_file")

    robot_description = visual_robot_urdf.read_text(encoding="utf-8")
    gzserver_cmd = [
        "gzserver",
        world_file,
        "-s",
        "libgazebo_ros_init.so",
        "-s",
        "libgazebo_ros_factory.so",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="true"),
            DeclareLaunchArgument("client_delay", default_value="1.0"),
            DeclareLaunchArgument("spawn_delay", default_value="4.0"),
            DeclareLaunchArgument("world_file", default_value=str(default_world_file)),
            DeclareLaunchArgument("x", default_value="-19.0"),
            DeclareLaunchArgument("y", default_value="1.0"),
            DeclareLaunchArgument("z", default_value="0.59"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("gazebo_robot_urdf", default_value=str(default_gazebo_robot_urdf)),
            DeclareLaunchArgument("publish_standing_joints", default_value="false"),
            DeclareLaunchArgument("gazebo_master_uri", default_value="http://127.0.0.1:11345"),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            SetEnvironmentVariable("GAZEBO_MASTER_URI", gazebo_master_uri),
            SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
            SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", gazebo_plugin_path),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="m20_robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"robot_description": robot_description},
                ],
            ),
            Node(
                package="m20_industrial_inspection_gazebo",
                executable="m20_standing_joint_state_publisher",
                name="m20_standing_joint_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"output_topic": "/joint_states"},
                ],
                condition=IfCondition(publish_standing_joints),
            ),
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
                        cmd=["gzclient", "--verbose"],
                        output="screen",
                        condition=IfCondition(gui),
                    )
                ],
            ),
            TimerAction(
                period=spawn_delay,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "/usr/bin/python3",
                            "/opt/ros/humble/lib/gazebo_ros/spawn_entity.py",
                            "-file",
                            gazebo_robot_urdf,
                            "-entity",
                            "m20",
                            "-x",
                            x,
                            "-y",
                            y,
                            "-z",
                            z,
                            "-Y",
                            yaw,
                        ],
                        name="spawn_m20",
                        output="screen",
                    )
                ],
            ),
        ]
    )
