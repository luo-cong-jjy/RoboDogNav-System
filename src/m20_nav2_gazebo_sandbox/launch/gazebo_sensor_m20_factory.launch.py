import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_gazebo_model_path(pkg_share: Path) -> str:
    """Expose only this package as a Gazebo model path for model:// mesh URIs."""
    model_path_root = Path("/tmp") / f"m20_nav2_gazebo_model_path_{os.getpid()}"
    model_path_root.mkdir(parents=True, exist_ok=True)
    model_link = model_path_root / "m20_nav2_gazebo_sandbox"
    if not model_link.exists():
        model_link.symlink_to(pkg_share, target_is_directory=True)
    return str(model_path_root)


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("m20_nav2_gazebo_sandbox"))
    pkg_prefix = Path(get_package_prefix("m20_nav2_gazebo_sandbox"))
    default_world = str(pkg_share / "worlds" / "factory_environment_v2.world")
    gazebo_robot_urdf = pkg_share / "models" / "m20_gazebo_combined_rslidar3d.urdf"
    visual_robot_urdf = pkg_share / "models" / "urdf" / "M20_nav_visual.urdf"
    pointcloud_to_scan_launch = pkg_share / "launch" / "rslidar_pointcloud_to_scan.launch.py"
    rviz_config = str(pkg_share / "rviz" / "nav2_sandbox.rviz")
    plugin_path = str(pkg_prefix / "lib")
    existing_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    gazebo_plugin_path = (
        plugin_path
        if not existing_plugin_path
        else f"{plugin_path}:{existing_plugin_path}"
    )
    gazebo_model_path = _make_gazebo_model_path(pkg_share)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_pointcloud_to_scan = LaunchConfiguration("use_pointcloud_to_scan")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    gazebo_gui_delay = LaunchConfiguration("gazebo_gui_delay")
    spawn_delay = LaunchConfiguration("spawn_delay")
    rviz_delay = LaunchConfiguration("rviz_delay")
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    # Gazebo 使用四轮站立形态 M20；RViz 使用带雷达的官方外观模型。
    robot_description = visual_robot_urdf.read_text(encoding="utf-8")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_pointcloud_to_scan", default_value="true"),
        DeclareLaunchArgument("use_gazebo_gui", default_value="false"),
        DeclareLaunchArgument("gazebo_gui_delay", default_value="1.0"),
        DeclareLaunchArgument("spawn_delay", default_value="4.0"),
        DeclareLaunchArgument("rviz_delay", default_value="5.0"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.59"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("cloud_topic", default_value="/LIDAR/POINTS"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),

        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", gazebo_plugin_path),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pointcloud_to_scan_launch)),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "cloud_topic": cloud_topic,
                "scan_topic": scan_topic,
                "range_max": "30.0",
            }.items(),
            condition=IfCondition(use_pointcloud_to_scan),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"robot_description": robot_description},
            ],
        ),

        Node(
            package="m20_nav2_gazebo_sandbox",
            executable="m20_standing_joint_state_publisher",
            name="m20_standing_joint_state_publisher",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"output_topic": "/joint_states"},
            ],
        ),

        Node(
            package="m20_nav2_gazebo_sandbox",
            executable="goal_pose_restamper",
            name="goal_pose_restamper",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"input_topic": "/m20_nav2/goal_pose_raw"},
                {"output_topic": "/goal_pose"},
                {"use_zero_stamp": True},
            ],
        ),

        ExecuteProcess(
            cmd=[
                "gzserver",
                "--verbose",
                world,
                "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so",
            ],
            output="screen",
        ),

        TimerAction(
            period=gazebo_gui_delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "gzclient",
                        "--verbose",
                    ],
                    output="screen",
                    condition=IfCondition(use_gazebo_gui),
                ),
            ],
        ),

        TimerAction(
            period=spawn_delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "/usr/bin/python3",
                        "/opt/ros/humble/lib/gazebo_ros/spawn_entity.py",
                        "-file", str(gazebo_robot_urdf),
                        "-entity", "m20_nav_proxy",
                        "-x", x,
                        "-y", y,
                        "-z", z,
                        "-Y", yaw,
                    ],
                    name="spawn_m20_nav_proxy",
                    output="screen",
                ),
            ],
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
