# M20 Nav2 Unified Interface

`m20_nav2_system` is the single integration entry point. The Nav2 interface is shared,
but backend-owned resources are intentionally separate. Gazebo and MuJoCo launches do
not load each other's worlds, sensor bridges, robot descriptions, Nav2 profiles, RViz
configurations, or project-owned helper scripts.

Profiles:

- `factory_navigation.launch.py`: Gazebo factory + saved 2D map + AMCL + Nav2 + RViz; no mission is started.
- `factory_slam_navigation.launch.py`: Gazebo-only static mapping world + Gazebo sensors + slam_toolbox + Nav2.
- `factory_saved_map_navigation.launch.py`: Gazebo-only dynamic world + saved map + AMCL + Nav2.
- `factory_inspection_mission_gazebo.launch.py`: Gazebo inspection action client.
- `factory_inspection_mission_mujoco.launch.py`: MuJoCo inspection action client.
- `bag_validation.launch.py`: rosbag `/LIDAR/POINTS` + pointcloud-to-scan + local costmap.
- `pcd_validation.launch.py`: PCD slice + pointcloud-to-scan + local costmap.

The stable topic boundary is `/scan` + `/odom` + `/tf -> Nav2 -> /cmd_vel`. Gazebo owns
its sensor/model bridge and MuJoCo owns its code-simulated sensor/SDK bridge. For a real
robot, replace only those backend topics before the safety and motion adapter.
