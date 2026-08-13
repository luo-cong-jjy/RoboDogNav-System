from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    playback_rate = LaunchConfiguration("playback_rate")
    play_bag = LaunchConfiguration("play_bag")
    use_rviz = LaunchConfiguration("use_rviz")

    package_share = FindPackageShare("m20_lidar_to_scan")
    converter_launch = PathJoinSubstitution([
        package_share,
        "launch",
        "pointcloud_to_scan.launch.py",
    ])
    costmap_params = PathJoinSubstitution([
        package_share,
        "config",
        "nav2_costmap_from_bag.yaml",
    ])
    rviz_config = PathJoinSubstitution([
        package_share,
        "rviz",
        "lidar_to_scan.rviz",
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
        DeclareLaunchArgument("use_rviz", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(converter_launch),
            launch_arguments={
                "use_sim_time": "true",
                "cloud_topic": "/LIDAR/POINTS",
                "scan_topic": "/scan",
                "target_frame": "",
            }.items(),
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_odom_to_base_link",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_base_link_to_lidar_link",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "lidar_link"],
        ),

        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            output="screen",
            parameters=[costmap_params],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_local_costmap",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "bond_timeout": 30.0,
                "node_names": ["costmap/costmap"],
            }],
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
                "--loop",
            ],
            output="screen",
            emulate_tty=True,
        ),

        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2_nav2_costmap_from_bag",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
        ),
    ])
