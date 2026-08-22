# M20 Nav2 Unified Interface

`m20_nav2_system` is the single integration entry point. It intentionally reuses the
existing Gazebo and point-cloud packages through launch composition; their models and
algorithms remain independently replaceable.

Profiles:

- `factory_navigation.launch.py`: Gazebo factory + selectable 2D/3D lidar + Nav2 + RViz; no mission is started.
- `factory_inspection_mission.launch.py`: starts the inspection action client after free navigation is ready.
- `bag_validation.launch.py`: rosbag `/LIDAR/POINTS` + pointcloud-to-scan + local costmap.
- `pcd_validation.launch.py`: PCD slice + pointcloud-to-scan + local costmap.

The stable topic boundary is `/LIDAR/POINTS -> /scan -> Nav2 -> /cmd_vel`. For a real
robot, remap the Nav2 output to `/m20_nav2/cmd_vel` before the safety and motion adapter.
