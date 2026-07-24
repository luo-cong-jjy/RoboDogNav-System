from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")
    package_share = FindPackageShare("m20_lidar_to_scan")
    planner_params = PathJoinSubstitution([
        package_share,
        "config",
        "pcd_nav2_planner.yaml",
    ])
    default_map_yaml = PathJoinSubstitution([
        package_share,
        "maps",
        "office4f",
        "t100ipro_grid.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", default_value=default_map_yaml),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                planner_params,
                {
                    "use_sim_time": False,
                    "yaml_filename": ParameterValue(map_yaml, value_type=str),
                },
            ],
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[planner_params],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_base_link_for_planner_test",
            arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
        ),

        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="m20_lidar_to_scan",
                    executable="lifecycle_configure_activate",
                    name="activate_pcd_map_server",
                    output="screen",
                    parameters=[{
                        "target_node": "/map_server",
                        "service_timeout_sec": 15.0,
                        "transition_timeout_sec": 20.0,
                        "activate_delay_sec": 0.5,
                    }],
                ),
            ],
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="m20_lidar_to_scan",
                    executable="lifecycle_configure_activate",
                    name="activate_pcd_planner_server",
                    output="screen",
                    parameters=[{
                        "target_node": "/planner_server",
                        "service_timeout_sec": 30.0,
                        "transition_timeout_sec": 60.0,
                        "activate_delay_sec": 0.5,
                    }],
                ),
            ],
        ),
    ])
