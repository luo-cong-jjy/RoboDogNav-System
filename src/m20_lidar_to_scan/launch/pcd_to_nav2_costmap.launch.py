from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pcd_path = LaunchConfiguration("pcd_path")
    pcd_frame = LaunchConfiguration("pcd_frame")
    publish_hz = LaunchConfiguration("publish_hz")
    max_points = LaunchConfiguration("max_points")
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
    default_pcd_path = PathJoinSubstitution([
        package_share,
        "maps",
        "office4f",
        "t100ipro_2026-07-10-15-20-18filtermap.pcd",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("pcd_path", default_value=default_pcd_path),
        DeclareLaunchArgument("pcd_frame", default_value="lidar_link"),
        DeclareLaunchArgument("publish_hz", default_value="1.0"),
        DeclareLaunchArgument("max_points", default_value="80000"),
        DeclareLaunchArgument("use_rviz", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(converter_launch),
            launch_arguments={
                "use_sim_time": "false",
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
            parameters=[costmap_params, {"use_sim_time": False}],
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="m20_lidar_to_scan",
                    executable="lifecycle_configure_activate",
                    name="activate_pcd_costmap",
                    output="screen",
                    parameters=[{
                        "target_node": "/costmap/costmap",
                        "service_timeout_sec": 15.0,
                        "transition_timeout_sec": 20.0,
                        "activate_delay_sec": 1.0,
                    }],
                ),
            ],
        ),

        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package="m20_lidar_to_scan",
                    executable="pcd_slice_publisher",
                    name="office4f_pcd_slice_publisher",
                    output="screen",
                    parameters=[{
                        "pcd_path": ParameterValue(pcd_path, value_type=str),
                        "topic": "/LIDAR/POINTS",
                        "frame_id": ParameterValue(pcd_frame, value_type=str),
                        "publish_hz": ParameterValue(publish_hz, value_type=float),
                        "min_height": -0.30,
                        "max_height": 0.30,
                        "range_min": 0.20,
                        "range_max": 30.0,
                        "max_points": ParameterValue(max_points, value_type=int),
                    }],
                ),
            ],
        ),

        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2_nav2_costmap_from_pcd",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": False}],
        ),
    ])
