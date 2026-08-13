from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    target_frame = LaunchConfiguration("target_frame")
    min_height = LaunchConfiguration("min_height")
    max_height = LaunchConfiguration("max_height")
    range_min = LaunchConfiguration("range_min")
    range_max = LaunchConfiguration("range_max")
    publish_static_lidar_tf = LaunchConfiguration("publish_static_lidar_tf")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_roll = LaunchConfiguration("lidar_roll")
    lidar_pitch = LaunchConfiguration("lidar_pitch")
    lidar_yaw = LaunchConfiguration("lidar_yaw")
    use_rviz = LaunchConfiguration("use_rviz")

    package_share = FindPackageShare("m20_foxy_nav_deploy")
    rviz_config = PathJoinSubstitution([
        package_share,
        "rviz",
        "m20_nav2_foxy.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("cloud_topic", default_value="/LIDAR/POINTS"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("min_height", default_value="-0.30"),
        DeclareLaunchArgument("max_height", default_value="0.30"),
        DeclareLaunchArgument("range_min", default_value="0.20"),
        DeclareLaunchArgument("range_max", default_value="30.0"),
        DeclareLaunchArgument("publish_static_lidar_tf", default_value="false"),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.0"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),

        Node(
            condition=IfCondition(publish_static_lidar_tf),
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_base_link_to_lidar_link",
            arguments=[
                lidar_x,
                lidar_y,
                lidar_z,
                lidar_yaw,
                lidar_pitch,
                lidar_roll,
                "base_link",
                "lidar_link",
            ],
        ),

        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="rslidar_pointcloud_to_scan",
            output="screen",
            remappings=[
                ("cloud_in", cloud_topic),
                ("scan", scan_topic),
            ],
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "target_frame": ParameterValue(target_frame, value_type=str),
                "transform_tolerance": 0.05,
                "min_height": ParameterValue(min_height, value_type=float),
                "max_height": ParameterValue(max_height, value_type=float),
                "angle_min": -3.141592653589793,
                "angle_max": 3.141592653589793,
                "angle_increment": 0.0034906585,
                "scan_time": 0.10,
                "range_min": ParameterValue(range_min, value_type=float),
                "range_max": ParameterValue(range_max, value_type=float),
                "use_inf": True,
            }],
        ),

        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2_m20_lidar_to_scan",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        ),
    ])
