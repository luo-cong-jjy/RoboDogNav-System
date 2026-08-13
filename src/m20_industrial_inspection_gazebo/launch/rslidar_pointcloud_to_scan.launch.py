from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    target_frame = LaunchConfiguration("target_frame")
    min_height = LaunchConfiguration("min_height")
    max_height = LaunchConfiguration("max_height")
    angle_min = LaunchConfiguration("angle_min")
    angle_max = LaunchConfiguration("angle_max")
    angle_increment = LaunchConfiguration("angle_increment")
    scan_time = LaunchConfiguration("scan_time")
    range_min = LaunchConfiguration("range_min")
    range_max = LaunchConfiguration("range_max")
    use_inf = LaunchConfiguration("use_inf")
    transform_tolerance = LaunchConfiguration("transform_tolerance")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("cloud_topic", default_value="/LIDAR/POINTS"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("target_frame", default_value=""),
        DeclareLaunchArgument("min_height", default_value="-0.30"),
        DeclareLaunchArgument("max_height", default_value="0.30"),
        DeclareLaunchArgument("angle_min", default_value="-3.141592653589793"),
        DeclareLaunchArgument("angle_max", default_value="3.141592653589793"),
        DeclareLaunchArgument("angle_increment", default_value="0.0034906585"),
        DeclareLaunchArgument("scan_time", default_value="0.10"),
        DeclareLaunchArgument("range_min", default_value="0.20"),
        DeclareLaunchArgument("range_max", default_value="30.0"),
        DeclareLaunchArgument("use_inf", default_value="true"),
        DeclareLaunchArgument("transform_tolerance", default_value="0.01"),

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
                "transform_tolerance": ParameterValue(transform_tolerance, value_type=float),
                "min_height": ParameterValue(min_height, value_type=float),
                "max_height": ParameterValue(max_height, value_type=float),
                "angle_min": ParameterValue(angle_min, value_type=float),
                "angle_max": ParameterValue(angle_max, value_type=float),
                "angle_increment": ParameterValue(angle_increment, value_type=float),
                "scan_time": ParameterValue(scan_time, value_type=float),
                "range_min": ParameterValue(range_min, value_type=float),
                "range_max": ParameterValue(range_max, value_type=float),
                "use_inf": ParameterValue(use_inf, value_type=bool),
            }],
        ),
    ])
