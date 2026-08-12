"""Native ROS 2 simulation for the M20-adapted SCAN-Planner setup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    if _as_bool(LaunchConfiguration("is_real_world").perform(context)):
        return []

    use_pcd_map = _as_bool(LaunchConfiguration("use_pcd_map").perform(context))
    use_gpu = _as_bool(LaunchConfiguration("use_gpu").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    pcd_map_file = LaunchConfiguration("pcd_map_file").perform(context)
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    body_pose_topic = LaunchConfiguration("body_pose_topic").perform(context)
    cloud_topic = LaunchConfiguration("cloud_topic").perform(context)
    sensor_cloud_topic = LaunchConfiguration("sensor_cloud_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    dyn_cloud_topic = LaunchConfiguration("dyn_cloud_topic").perform(context)
    uav_cloud_topic = LaunchConfiguration("uav_cloud_topic").perform(context)
    pose_topic = LaunchConfiguration("pose_topic").perform(context)
    path_topic = LaunchConfiguration("path_topic").perform(context)
    velocity_topic = LaunchConfiguration("velocity_topic").perform(context)
    trajectory_topic = LaunchConfiguration("trajectory_topic").perform(context)
    robot_marker_topic = LaunchConfiguration("robot_marker_topic").perform(context)
    height_topic = LaunchConfiguration("height_topic").perform(context)
    odom_child_frame_id = LaunchConfiguration("odom_child_frame_id").perform(context)
    map_x = float(LaunchConfiguration("map_size_x").perform(context))
    map_y = float(LaunchConfiguration("map_size_y").perform(context))
    map_z = float(LaunchConfiguration("map_size_z").perform(context))

    if use_pcd_map and (not pcd_map_file or not os.path.isfile(pcd_map_file)):
        raise RuntimeError(
            "use_pcd_map=true requires pcd_map_file to reference an existing PCD file"
        )

    planner_share = get_package_share_directory("m20_scan_planner")
    simulator_yaml = os.path.join(planner_share, "config", "simulator.yaml")
    nodes = []

    if use_pcd_map:
        nodes.append(
            Node(
                package="map_generator",
                executable="map_pub",
                name="map_pub",
                namespace="map_generator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "file_name": pcd_map_file,
                        "frame_id": "world",
                        "publish_rate": 0.2,
                        "downsample_res": 0.1,
                    }
                ],
            )
        )
    else:
        nodes.append(
            Node(
                package="mockamap",
                executable="mockamap_node",
                name="mockamap_node",
                output="screen",
                parameters=[
                    simulator_yaml,
                    {
                        "use_sim_time": use_sim_time,
                        "x_length": int(map_x),
                        "y_length": int(map_y),
                        "z_length": int(map_z),
                    },
                ],
                remappings=[("mock_map", "/map_generator/global_cloud")],
            )
        )

    nodes.extend(
        [
            Node(
                package="local_sensing_node",
                executable="opengl_render_node" if use_gpu else "pcl_render_node",
                name="pcl_render_node",
                output="screen",
                parameters=[
                    simulator_yaml,
                    {
                        "use_sim_time": use_sim_time,
                        "sensor_type": sensor_type,
                        "body_pose_topic": body_pose_topic,
                        "map.x_size": map_x,
                        "map.y_size": map_y,
                        "map.z_size": map_z,
                        "use_global_map_topic": True,
                        "pcd_map_file": pcd_map_file,
                    },
                ],
                remappings=[
                    ("global_map", "/map_generator/global_cloud"),
                    ("body_pose", body_pose_topic),
                    ("cloud", cloud_topic),
                    ("sensor_cloud", sensor_cloud_topic),
                    ("depth", depth_topic),
                    ("dyn_cloud", dyn_cloud_topic),
                    ("uav_cloud", uav_cloud_topic),
                ],
            ),
            Node(
                package="odom_visualization",
                executable="odom_visualization",
                name="odom_visualization",
                output="screen",
                parameters=[
                    simulator_yaml,
                    {
                        "use_sim_time": use_sim_time,
                        "child_frame_id": odom_child_frame_id,
                    },
                ],
                remappings=[
                    ("body_pose", body_pose_topic),
                    ("pose", pose_topic),
                    ("path", path_topic),
                    ("velocity", velocity_topic),
                    ("trajectory", trajectory_topic),
                    ("robot", robot_marker_topic),
                    ("height", height_topic),
                ],
            ),
        ]
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("body_pose_topic", default_value="/quad_0/body_pose"),
            DeclareLaunchArgument("cloud_topic", default_value="/quad_0/cloud"),
            DeclareLaunchArgument("sensor_cloud_topic", default_value="/quad_0/sensor_cloud"),
            DeclareLaunchArgument("depth_topic", default_value="/quad_0/depth"),
            DeclareLaunchArgument("dyn_cloud_topic", default_value="/quad_0/dyn_cloud"),
            DeclareLaunchArgument("uav_cloud_topic", default_value="/quad_0/uav_cloud"),
            DeclareLaunchArgument("pose_topic", default_value="/quad_0/pose"),
            DeclareLaunchArgument("path_topic", default_value="/quad_0/path"),
            DeclareLaunchArgument("velocity_topic", default_value="/quad_0/velocity"),
            DeclareLaunchArgument("trajectory_topic", default_value="/quad_0/trajectory"),
            DeclareLaunchArgument("robot_marker_topic", default_value="/quad_0/robot"),
            DeclareLaunchArgument("height_topic", default_value="/quad_0/height"),
            DeclareLaunchArgument("odom_child_frame_id", default_value="base"),
            OpaqueFunction(function=_setup),
        ]
    )
