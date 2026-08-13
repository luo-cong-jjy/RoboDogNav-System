from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_lidar_to_scan = LaunchConfiguration("use_lidar_to_scan")
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
    nav2_bringup_share = FindPackageShare("nav2_bringup")
    default_params = PathJoinSubstitution([
        package_share,
        "config",
        "m20_nav2_foxy_params.yaml",
    ])
    default_map = PathJoinSubstitution([
        package_share,
        "maps",
        "m20_site_map.yaml",
    ])
    rviz_config = PathJoinSubstitution([
        package_share,
        "rviz",
        "m20_nav2_foxy.rviz",
    ])
    nav2_bringup_launch = PathJoinSubstitution([
        nav2_bringup_share,
        "launch",
        "bringup_launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_namespace", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_lidar_to_scan", default_value="true"),
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
            condition=IfCondition(use_lidar_to_scan),
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": use_namespace,
                "slam": "False",
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": autostart,
            }.items(),
        ),

        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2_m20_nav2_foxy",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        ),
    ])
