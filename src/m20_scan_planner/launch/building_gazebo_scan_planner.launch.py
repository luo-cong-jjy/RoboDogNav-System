"""Run SCAN-Planner against the Gazebo Building world with the M20 model."""

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
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    pcd_map_file = LaunchConfiguration("pcd_map_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    controller_forward_only = _as_bool(LaunchConfiguration("controller_forward_only").perform(context))
    controller_heading_error_threshold = float(
        LaunchConfiguration("controller_heading_error_threshold").perform(context)
    )
    controller_max_forward_lateral_speed = float(
        LaunchConfiguration("controller_max_forward_lateral_speed").perform(context)
    )
    use_traversability_global_planner = _as_bool(
        LaunchConfiguration("use_traversability_global_planner").perform(context)
    )
    use_support_path_follower = _as_bool(
        LaunchConfiguration("use_support_path_follower").perform(context)
    )
    body_height = float(LaunchConfiguration("traversability_body_height").perform(context))
    inflation_z_up = float(LaunchConfiguration("building_obstacles_inflation_z_up").perform(context))
    inflation_z_down = float(LaunchConfiguration("building_obstacles_inflation_z_down").perform(context))

    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )
    if use_traversability_global_planner and navi_mode != 3:
        raise RuntimeError(
            "use_traversability_global_planner=true requires navi_mode:=3 so "
            "SCAN-Planner subscribes to /initial_path"
        )
    if use_support_path_follower and not use_traversability_global_planner:
        raise RuntimeError(
            "use_support_path_follower=true requires use_traversability_global_planner:=true"
        )

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
        "grid_map.body_height": body_height,
        "grid_map.obstacles_inflation_z_up": inflation_z_up,
        "grid_map.obstacles_inflation_z_down": inflation_z_down,
        "grid_map.sliding_map_size_z": float(
            LaunchConfiguration("building_sliding_map_size_z").perform(context)
        ),
        "grid_map.local_update_range_z": float(
            LaunchConfiguration("building_local_update_range_z").perform(context)
        ),
    }
    controller_overrides = {
        **common,
        "forward_only": controller_forward_only,
        "heading_error_threshold": controller_heading_error_threshold,
        "max_forward_lateral_speed": controller_max_forward_lateral_speed,
    }

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planner_share, "launch", "building_m20_gazebo.launch.py")
            ),
            launch_arguments={
                name: LaunchConfiguration(name)
                for name in (
                    "use_sim_time",
                    "gui",
                    "verbose",
                    "client_delay",
                    "spawn_delay",
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
                "use_pcd_map": LaunchConfiguration("use_pcd_map"),
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
    if use_support_path_follower:
        actions.append(
            Node(
                package="m20_scan_planner",
                executable="support_path_follower.py",
                name="m20_support_path_follower",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "lookahead_distance": float(
                            LaunchConfiguration("support_follower_lookahead_distance").perform(context)
                        ),
                        "waypoint_reached_distance": float(
                            LaunchConfiguration("support_follower_waypoint_reached_distance").perform(context)
                        ),
                        "finish_distance": float(
                            LaunchConfiguration("support_follower_finish_distance").perform(context)
                        ),
                        "max_vx": float(LaunchConfiguration("support_follower_max_vx").perform(context)),
                        "max_vy": float(LaunchConfiguration("support_follower_max_vy").perform(context)),
                        "max_vyaw": float(
                            LaunchConfiguration("support_follower_max_vyaw").perform(context)
                        ),
                    }
                ],
                remappings=[
                    ("initial_path", "/initial_path"),
                    ("body_pose", body_pose),
                    ("cmd_vel", "/cmd_vel"),
                ],
            )
        )
    else:
        actions.extend(
            [
                Node(
                    package="m20_scan_planner",
                    executable="scan_planner_node",
                    name="scan_planner_node",
                    output="screen",
                    parameters=[planner_yaml]
                    + ([keypoints_file] if keypoints_file else [])
                    + [planner_overrides],
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
            ]
        )
    if use_traversability_global_planner:
        actions.append(
            Node(
                package="m20_scan_planner",
                executable="traversability_global_planner.py",
                name="m20_traversability_global_planner",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "pcd_map_file": pcd_map_file,
                        "frame_id": "world",
                        "grid_resolution": float(
                            LaunchConfiguration("traversability_grid_resolution").perform(context)
                        ),
                        "height_bin_resolution": float(
                            LaunchConfiguration("traversability_height_bin_resolution").perform(context)
                        ),
                        "step_max": float(LaunchConfiguration("traversability_step_max").perform(context)),
                        "max_edge_slope": float(
                            LaunchConfiguration("traversability_max_edge_slope").perform(context)
                        ),
                        "body_height": body_height,
                        "goal_layer_policy": LaunchConfiguration(
                            "traversability_goal_layer_policy"
                        ).perform(context),
                        "waypoint_spacing": float(
                            LaunchConfiguration("traversability_waypoint_spacing").perform(context)
                        ),
                        "vertical_waypoint_spacing": float(
                            LaunchConfiguration("traversability_vertical_waypoint_spacing").perform(context)
                        ),
                        "max_output_waypoints": int(
                            LaunchConfiguration("traversability_max_output_waypoints").perform(context)
                        ),
                        "max_goal_candidates": int(
                            LaunchConfiguration("traversability_max_goal_candidates").perform(context)
                        ),
                    }
                ],
                remappings=[
                    ("body_pose", body_pose),
                    ("move_base_simple/goal", "/move_base_simple/goal"),
                    ("clicked_point", "/clicked_point"),
                    ("initial_path", "/initial_path"),
                ],
            )
        )
    return actions


def generate_launch_description():
    default_building_pcd = os.path.join(
        get_package_share_directory("m20_scan_planner"),
        "models",
        "1_Building",
        "Building_surface_0p10.pcd",
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
            DeclareLaunchArgument("spawn_delay", default_value="8.0"),
            DeclareLaunchArgument("x", default_value="-5.0"),
            DeclareLaunchArgument("y", default_value="7.0"),
            DeclareLaunchArgument("z", default_value="0.59"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("gazebo_robot_urdf", default_value=default_gazebo_robot_urdf),
            DeclareLaunchArgument("publish_standing_joints", default_value="false"),
            DeclareLaunchArgument("gazebo_master_uri", default_value="http://127.0.0.1:11345"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="true"),
            DeclareLaunchArgument("pcd_map_file", default_value=default_building_pcd),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="8.0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_delay", default_value="3.0"),
            DeclareLaunchArgument("controller_forward_only", default_value="true"),
            DeclareLaunchArgument("controller_heading_error_threshold", default_value="0.35"),
            DeclareLaunchArgument("controller_max_forward_lateral_speed", default_value="0.05"),
            DeclareLaunchArgument("use_traversability_global_planner", default_value="false"),
            DeclareLaunchArgument("traversability_grid_resolution", default_value="0.20"),
            DeclareLaunchArgument("traversability_height_bin_resolution", default_value="0.08"),
            DeclareLaunchArgument("traversability_step_max", default_value="0.30"),
            DeclareLaunchArgument("traversability_max_edge_slope", default_value="1.20"),
            DeclareLaunchArgument("traversability_body_height", default_value="0.59"),
            DeclareLaunchArgument("building_obstacles_inflation_z_up", default_value="0.15"),
            DeclareLaunchArgument("building_obstacles_inflation_z_down", default_value="0.20"),
            DeclareLaunchArgument("building_sliding_map_size_z", default_value="10.0"),
            DeclareLaunchArgument("building_local_update_range_z", default_value="5.0"),
            DeclareLaunchArgument("traversability_goal_layer_policy", default_value="nearest_to_start_height"),
            DeclareLaunchArgument("traversability_waypoint_spacing", default_value="0.80"),
            DeclareLaunchArgument("traversability_vertical_waypoint_spacing", default_value="0.25"),
            DeclareLaunchArgument("traversability_max_output_waypoints", default_value="28"),
            DeclareLaunchArgument("traversability_max_goal_candidates", default_value="16"),
            DeclareLaunchArgument("use_support_path_follower", default_value="false"),
            DeclareLaunchArgument("support_follower_lookahead_distance", default_value="0.70"),
            DeclareLaunchArgument("support_follower_waypoint_reached_distance", default_value="0.35"),
            DeclareLaunchArgument("support_follower_finish_distance", default_value="0.40"),
            DeclareLaunchArgument("support_follower_max_vx", default_value="0.45"),
            DeclareLaunchArgument("support_follower_max_vy", default_value="0.08"),
            DeclareLaunchArgument("support_follower_max_vyaw", default_value="0.8"),
            OpaqueFunction(function=_setup),
        ]
    )
