"""Run single-floor synchronized Gazebo + PCD map + SCAN-Planner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    planner_share = get_package_share_directory("m20_scan_planner")
    planner_yaml = os.path.join(planner_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(planner_share, "config", "controllers.yaml")

    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    pcd_map_file = LaunchConfiguration("pcd_map_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    controller_forward_only = _as_bool(LaunchConfiguration("controller_forward_only").perform(context))
    controller_heading_error_threshold = float(
        LaunchConfiguration("controller_heading_error_threshold").perform(context)
    )
    controller_max_forward_lateral_speed = float(
        LaunchConfiguration("controller_max_forward_lateral_speed").perform(context)
    )
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)

    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )
    if not pcd_map_file or not os.path.isfile(pcd_map_file):
        raise RuntimeError("pcd_map_file must reference the synchronized single-floor PCD")

    body_pose = "/odom"
    sensor_pose = "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
    cloud = "/quad_0/cloud"
    depth = "/quad_0/depth"

    common = {"use_sim_time": use_sim_time}
    planner_overrides = {
        **common,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": True,
        "grid_map.need_extrinsic": False,
    }
    controller_overrides = {
        **common,
        "forward_only": controller_forward_only,
        "heading_error_threshold": controller_heading_error_threshold,
        "max_forward_lateral_speed": controller_max_forward_lateral_speed,
    }

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planner_share, "launch", "mockamap_m20_gazebo.launch.py")
            ),
            launch_arguments={
                name: LaunchConfiguration(name)
                for name in (
                    "use_sim_time",
                    "gui",
                    "verbose",
                    "client_delay",
                    "spawn_delay",
                    "world_file",
                    "x",
                    "y",
                    "z",
                    "yaw",
                    "gazebo_robot_urdf",
                    "publish_standing_joints",
                    "gazebo_master_uri",
                )
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planner_share, "launch", "simulator.launch.py")
            ),
            launch_arguments={
                "is_real_world": "false",
                "sensor_type": LaunchConfiguration("sensor_type"),
                "use_gpu": LaunchConfiguration("use_gpu"),
                "use_pcd_map": "true",
                "pcd_map_file": LaunchConfiguration("pcd_map_file"),
                "map_size_x": LaunchConfiguration("map_size_x"),
                "map_size_y": LaunchConfiguration("map_size_y"),
                "map_size_z": LaunchConfiguration("map_size_z"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "body_pose_topic": body_pose,
                "cloud_topic": cloud,
                "sensor_cloud_topic": "/quad_0/sensor_cloud",
                "depth_topic": depth,
                "dyn_cloud_topic": "/quad_0/dyn_cloud",
                "uav_cloud_topic": "/quad_0/uav_cloud",
                "pose_topic": "/quad_0/pose",
                "path_topic": "/quad_0/path",
                "velocity_topic": "/quad_0/velocity",
                "trajectory_topic": "/quad_0/trajectory",
                "robot_marker_topic": "/quad_0/robot",
                "height_topic": "/quad_0/height",
                "odom_child_frame_id": "base_link",
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_odom_tf",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        ),
        Node(
            package="m20_scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml] + ([keypoints_file] if keypoints_file else []) + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/initial_path"),
            ],
        ),
        Node(
            package="m20_scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            parameters=[controllers_yaml, controller_overrides],
            remappings=[
                ("planning/bspline", "/planning/bspline"),
                ("body_pose", body_pose),
                ("cmd_vel", "/cmd_vel"),
            ],
        ),
        TimerAction(
            period=LaunchConfiguration("rviz_delay"),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(planner_share, "launch", "rviz.launch.py")
                    ),
                    launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
                    condition=IfCondition(LaunchConfiguration("use_rviz")),
                )
            ],
        ),
    ]


def generate_launch_description():
    default_map_dir = os.path.join(
        get_package_share_directory("m20_scan_planner"),
        "models",
        "mockamap_single_floor",
    )
    default_gazebo_robot_urdf = os.path.join(
        get_package_share_directory("m20_scan_planner"),
        "models",
        "m20",
        "m20_gazebo_legged_no_lidar.urdf",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="true"),
            DeclareLaunchArgument("client_delay", default_value="1.0"),
            DeclareLaunchArgument("spawn_delay", default_value="4.0"),
            DeclareLaunchArgument(
                "world_file",
                default_value=os.path.join(default_map_dir, "mockamap_single_floor.world"),
            ),
            DeclareLaunchArgument("x", default_value="-19.0"),
            DeclareLaunchArgument("y", default_value="1.0"),
            DeclareLaunchArgument("z", default_value="0.59"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("gazebo_robot_urdf", default_value=default_gazebo_robot_urdf),
            DeclareLaunchArgument("publish_standing_joints", default_value="false"),
            DeclareLaunchArgument("gazebo_master_uri", default_value="http://127.0.0.1:11345"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument(
                "pcd_map_file",
                default_value=os.path.join(default_map_dir, "mockamap_single_floor.pcd"),
            ),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_delay", default_value="3.0"),
            DeclareLaunchArgument("controller_forward_only", default_value="true"),
            DeclareLaunchArgument("controller_heading_error_threshold", default_value="0.60"),
            DeclareLaunchArgument("controller_max_forward_lateral_speed", default_value="0.15"),
            OpaqueFunction(function=_setup),
        ]
    )
