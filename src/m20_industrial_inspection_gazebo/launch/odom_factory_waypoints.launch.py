from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_WAYPOINTS = (
    "0,0;20,0;20,4;0,4;0,8;20,8;"
    "20,12;0,12;0,16;20,16;20,20;0,20"
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("waypoints", default_value=DEFAULT_WAYPOINTS),
        DeclareLaunchArgument("goal_tolerance", default_value="0.35"),
        DeclareLaunchArgument("max_linear", default_value="0.28"),
        DeclareLaunchArgument("max_angular", default_value="0.75"),
        DeclareLaunchArgument("loop", default_value="false"),

        Node(
            package="m20_industrial_inspection_gazebo",
            executable="odom_waypoint_follower",
            name="odom_waypoint_follower",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"waypoints": LaunchConfiguration("waypoints")},
                {"goal_tolerance": LaunchConfiguration("goal_tolerance")},
                {"max_linear": LaunchConfiguration("max_linear")},
                {"max_angular": LaunchConfiguration("max_angular")},
                {"loop": LaunchConfiguration("loop")},
            ],
        ),
    ])
