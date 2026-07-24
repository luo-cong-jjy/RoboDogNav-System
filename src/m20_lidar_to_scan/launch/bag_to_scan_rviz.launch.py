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
    publish_pcd_map = LaunchConfiguration("publish_pcd_map")
    pcd_map_file = LaunchConfiguration("pcd_map_file")
    pcd_map_frame = LaunchConfiguration("pcd_map_frame")
    pcd_map_period_ms = LaunchConfiguration("pcd_map_period_ms")
    publish_grid_map = LaunchConfiguration("publish_grid_map")
    grid_map_file = LaunchConfiguration("grid_map_file")
    grid_map_frame = LaunchConfiguration("grid_map_frame")
    use_rviz = LaunchConfiguration("use_rviz")

    package_share = FindPackageShare("m20_lidar_to_scan")
    converter_launch = PathJoinSubstitution([
        package_share,
        "launch",
        "pointcloud_to_scan.launch.py",
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
        DeclareLaunchArgument("playback_rate", default_value="0.2"),
        DeclareLaunchArgument("play_bag", default_value="true"),
        DeclareLaunchArgument("publish_pcd_map", default_value="true"),
        DeclareLaunchArgument(
            "pcd_map_file",
            default_value=PathJoinSubstitution([
                package_share,
                "maps",
                "office4f",
                "t100ipro_2026-07-10-15-20-18filtermap.pcd",
            ]),
        ),
        DeclareLaunchArgument("pcd_map_frame", default_value="lidar_link"),
        DeclareLaunchArgument("pcd_map_period_ms", default_value="1000"),
        DeclareLaunchArgument("publish_grid_map", default_value="true"),
        DeclareLaunchArgument(
            "grid_map_file",
            default_value=PathJoinSubstitution([
                package_share,
                "maps",
                "factory_slam",
                "factory_slam_map.yaml",
            ]),
        ),
        DeclareLaunchArgument("grid_map_frame", default_value="lidar_link"),
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
            condition=IfCondition(publish_pcd_map),
            package="pcl_ros",
            executable="pcd_to_pointcloud",
            name="office4f_global_map_publisher",
            output="screen",
            remappings=[
                ("cloud_pcd", "/office4f/global_map"),
            ],
            parameters=[{
                "file_name": ParameterValue(pcd_map_file, value_type=str),
                "tf_frame": ParameterValue(pcd_map_frame, value_type=str),
                "publishing_period_ms": ParameterValue(pcd_map_period_ms, value_type=int),
            }],
        ),

        Node(
            condition=IfCondition(publish_grid_map),
            package="nav2_map_server",
            executable="map_server",
            name="factory_slam_map_server",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "yaml_filename": ParameterValue(grid_map_file, value_type=str),
                "topic_name": "/map",
                "frame_id": ParameterValue(grid_map_frame, value_type=str),
            }],
        ),

        Node(
            condition=IfCondition(publish_grid_map),
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_factory_slam_map",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["factory_slam_map_server"],
            }],
        ),

        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2_lidar_to_scan",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
        ),
    ])
