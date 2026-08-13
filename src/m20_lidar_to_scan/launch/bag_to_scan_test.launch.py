from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    playback_rate = LaunchConfiguration("playback_rate")
    play_bag = LaunchConfiguration("play_bag")
    run_probe = LaunchConfiguration("run_probe")
    probe_samples = LaunchConfiguration("probe_samples")

    converter_launch = PathJoinSubstitution([
        FindPackageShare("m20_lidar_to_scan"),
        "launch",
        "pointcloud_to_scan.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_path",
            default_value=EnvironmentVariable(
                "M20_LIDAR_BAG_PATH",
                default_value="/home/virdyn/robodog_nav_system/datasets/m20",
            ),
        ),
        DeclareLaunchArgument("playback_rate", default_value="0.5"),
        DeclareLaunchArgument("play_bag", default_value="true"),
        DeclareLaunchArgument("run_probe", default_value="true"),
        DeclareLaunchArgument("probe_samples", default_value="5"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(converter_launch),
            launch_arguments={
                "use_sim_time": "true",
                "cloud_topic": "/LIDAR/POINTS",
                "scan_topic": "/scan",
                "target_frame": "",
            }.items(),
        ),

        ExecuteProcess(
            condition=IfCondition(play_bag),
            cmd=[
                "ros2",
                "bag",
                "play",
                bag_path,
                "--clock",
                "--rate",
                playback_rate,
            ],
            output="screen",
            emulate_tty=True,
        ),

        Node(
            condition=IfCondition(run_probe),
            package="m20_lidar_to_scan",
            executable="scan_probe",
            name="scan_probe",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "topic": "/scan",
                "samples": ParameterValue(probe_samples, value_type=int),
            }],
        ),
    ])
