from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("m20_nav2_gazebo_sandbox"),
        "launch",
        "gazebo_sensor_m20_factory.launch.py",
    ])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("m20_nav2_gazebo_sandbox"),
        "launch",
        "nav2_map_navigation_m20_factory.launch.py",
    ])
    mission_launch = PathJoinSubstitution([
        FindPackageShare("m20_nav2_gazebo_sandbox"),
        "launch",
        "factory_inspection_mission.launch.py",
    ])
    default_map = PathJoinSubstitution([
        FindPackageShare("m20_nav2_gazebo_sandbox"),
        "maps",
        "factory_slam_map.yaml",
    ])
    default_mission_params = PathJoinSubstitution([
        FindPackageShare("m20_nav2_gazebo_sandbox"),
        "config",
        "factory_inspection_midpoints.yaml",
    ])
    default_world = PathJoinSubstitution([
        FindPackageShare("m20_nav2_gazebo_sandbox"),
        "worlds",
        "factory_environment_v3_dynamic_obstacles.world",
    ])

    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    gazebo_gui_delay = LaunchConfiguration("gazebo_gui_delay")
    spawn_delay = LaunchConfiguration("spawn_delay")
    rviz_delay = LaunchConfiguration("rviz_delay")
    nav2_delay = LaunchConfiguration("nav2_delay")
    mission_delay = LaunchConfiguration("mission_delay")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    world = LaunchConfiguration("world")
    map_file = LaunchConfiguration("map")
    mission_params = LaunchConfiguration("mission_params")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gazebo_gui", default_value="true"),
        DeclareLaunchArgument("gazebo_gui_delay", default_value="1.0"),
        DeclareLaunchArgument("spawn_delay", default_value="4.0"),
        DeclareLaunchArgument("rviz_delay", default_value="5.0"),
        DeclareLaunchArgument("nav2_delay", default_value="7.0"),
        DeclareLaunchArgument("mission_delay", default_value="16.0"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.59"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("mission_params", default_value=default_mission_params),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "use_rviz": use_rviz,
                "use_gazebo_gui": use_gazebo_gui,
                "gazebo_gui_delay": gazebo_gui_delay,
                "spawn_delay": spawn_delay,
                "rviz_delay": rviz_delay,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "world": world,
            }.items(),
        ),

        TimerAction(
            period=nav2_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        "use_rviz": "false",
                        "map": map_file,
                    }.items(),
                ),
            ],
        ),

        TimerAction(
            period=mission_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(mission_launch),
                    launch_arguments={
                        "params_file": mission_params,
                    }.items(),
                ),
            ],
        ),
    ])
