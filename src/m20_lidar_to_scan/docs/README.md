# M20 3D PointCloud2 to 2D LaserScan

这个包用于验证 M20 背部 RoboSense 3D 雷达点云能否转换成 Nav2 可直接使用的二维 `/scan`。

## 数据

官方录包已复制到：

```bash
/home/virdyn/robodog_nav_system/datasets/m20
```

该 bag 包含：

- `/LIDAR/POINTS`: `sensor_msgs/msg/PointCloud2`
- `/IMU`: `sensor_msgs/msg/Imu`

地图资源已复制到本包：

```bash
src/m20_lidar_to_scan/maps/office4f/global.pcd
src/m20_lidar_to_scan/maps/office4f/0.pcd
src/m20_lidar_to_scan/maps/office4f/t100ipro_2026-07-10-15-20-18filtermap.pcd
src/m20_lidar_to_scan/maps/factory_slam/factory_slam_map.yaml
src/m20_lidar_to_scan/maps/factory_slam/factory_slam_map.pgm
```

其中 `office4f` 是官方 PCD 三维地图，用于当前 RViz 点云对比；`factory_slam` 是 Nav2 可用的二维静态地图。

## 编译

```bash
cd /home/virdyn/robodog_nav_system
source /opt/ros/humble/setup.bash
colcon build --packages-select m20_lidar_to_scan --symlink-install
source install/setup.bash
```

## 播放官方 bag 并转换成 /scan

```bash
ros2 launch m20_lidar_to_scan bag_to_scan_test.launch.py
```

默认会同时启动 `scan_probe`，打印几帧 `/scan` 的统计信息。如果能看到 `ranges` 和 `finite` 数量，就说明 3D 点云已经成功转换成 Nav2 可订阅的 `sensor_msgs/msg/LaserScan`。

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

## RViz 同屏查看原始点云和转换结果

```bash
export ROS_LOCALHOST_ONLY=1
ros2 launch m20_lidar_to_scan bag_to_scan_rviz.launch.py
```

RViz 中：

- `Occupancy Grid Map` 是从 `factory_slam_map.yaml/.pgm` 发布出的 `/map`
- `Official PCD Map` 是从官方 `office4f/global.pcd` 发布出的 `/office4f/global_map`
- `Raw 3D PointCloud` 是原始 `/LIDAR/POINTS`
- `Converted 2D LaserScan` 是转换后的 `/scan`

测试 launch 默认循环播放 bag，便于反复观察。当前只是可视化对比，PCD 地图和二维栅格地图默认都发布在 `lidar_link` 下；等后续接入定位后，再改成真实 `map -> lidar_link` TF。真机联调时不要设置 `ROS_LOCALHOST_ONLY=1`。

## 用官方 bag 验证 Nav2 local_costmap

这个测试不需要 Gazebo 世界，只验证官方 3D 点云转出的 `/scan` 能否进入 Nav2 obstacle layer 并生成局部代价地图。

```bash
export ROS_LOCALHOST_ONLY=1
ros2 launch m20_lidar_to_scan bag_to_nav2_costmap.launch.py
```

RViz 中重点看：

- `Converted 2D LaserScan`: 转换后的 `/scan`
- `Nav2 Local Costmap`: Nav2 obstacle layer 根据 `/scan` 生成的 `/costmap/costmap`

## 用 office4f 大 PCD 验证 Nav2 local_costmap

如果手里不是 rosbag，而是官方生成的 `.pcd` 点云地图，也可以走同一条链路：

```bash
export ROS_LOCALHOST_ONLY=1
ros2 launch m20_lidar_to_scan pcd_to_nav2_costmap.launch.py use_rviz:=false
```

默认使用：

```bash
src/m20_lidar_to_scan/maps/office4f/t100ipro_2026-07-10-15-20-18filtermap.pcd
```

由于这个 PCD 大约有 618 万个点，测试 launch 不会直接整包发布，而是通过 `pcd_slice_publisher` 做高度切片和点数限制：

```bash
z=[-0.30, 0.30]
range=[0.20, 30.00]
max_points=80000
```

本次测试结果：

```text
/LIDAR/POINTS: 约 1.0 Hz，发布 80000 个切片点
/scan: ranges=1801, finite=1653, min=0.21m, max=12.21m
/costmap/costmap: 约 1.667 Hz
```

如果要打开 RViz：

```bash
ros2 launch m20_lidar_to_scan pcd_to_nav2_costmap.launch.py use_rviz:=true
```

如果 RViz 里点云“一闪一闪”，通常不是 PCD 错，而是点云发布频率和 RViz `Decay Time` 不匹配。例如 `/LIDAR/POINTS` 以 1Hz 发布，但 RViz 只保留 0.3s，就会显示 0.3s、消失 0.7s。当前 RViz 配置已经把 `Raw 3D PointCloud` 的 `Decay Time` 调到 2s；也可以提高发布频率：

```bash
ros2 launch m20_lidar_to_scan pcd_to_nav2_costmap.launch.py \
  use_rviz:=true \
  publish_hz:=5.0 \
  max_points:=80000
```

也可以换其他 PCD：

```bash
ros2 launch m20_lidar_to_scan pcd_to_nav2_costmap.launch.py \
  pcd_path:=/path/to/your_map.pcd \
  max_points:=80000
```

## 从 PCD 构建二维栅格地图并跑规划

PCD 本身只是点云，不像 SLAM 地图一样天然包含 free space。这里的工程验证做法是：取指定高度范围内的点投影到 x-y 平面，标记 occupied cell，其余 cell 暂按 free 处理，用来验证全局规划链路。

生成 Nav2 可读取的 `.pgm + .yaml`：

```bash
python3 src/m20_lidar_to_scan/scripts/pcd_to_occupancy_grid \
  --pcd src/m20_lidar_to_scan/maps/office4f/t100ipro_2026-07-10-15-20-18filtermap.pcd \
  --output src/m20_lidar_to_scan/maps/office4f/t100ipro_grid.yaml \
  --resolution 0.20 \
  --min-height -0.30 \
  --max-height 0.30 \
  --inflate-radius 0.30 \
  --min-points-per-cell 2
```

本次生成结果：

```text
slice points: 958179
map: 514 x 1099 cells, resolution=0.200 m
origin: [-72.152, -132.405, 0.000]
occupied cells: 90975/564886 (16.11%)
```

先用轻量 A* 检查地图连通性：

```bash
python3 src/m20_lidar_to_scan/scripts/grid_astar_planner \
  --map src/m20_lidar_to_scan/maps/office4f/t100ipro_grid.yaml \
  --output-path src/m20_lidar_to_scan/maps/office4f/t100ipro_grid_astar_path.csv \
  --overlay src/m20_lidar_to_scan/maps/office4f/t100ipro_grid_astar_overlay.ppm
```

本次 A* 结果：

```text
start: (-51.652, -77.505)
goal: (10.148, 32.495)
path points: 773
path length: 174.37 m
```

再用 Nav2 的 `NavfnPlanner` 跑全局规划：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch m20_lidar_to_scan pcd_nav2_planner.launch.py
```

另开一个终端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run m20_lidar_to_scan compute_path_probe
```

本次 Nav2 `ComputePathToPose` 结果：

```text
poses=1680
length=168.19m
planning_time=0.0147s
```

注意：这只是“PCD 生成地图 + 全局路径规划”验证，不是完整导航闭环。完整闭环还需要定位、`map -> odom -> base_link` TF、里程计、controller、速度限制和 `/cmd_vel` 执行链。

## 真机实时转换

真机或随车 x86 上已经能看到 `/LIDAR/POINTS` 后，启动：

```bash
ros2 launch m20_lidar_to_scan pointcloud_to_scan.launch.py
```

如果已经发布雷达到 `base_link` 的 TF，可以加：

```bash
ros2 launch m20_lidar_to_scan pointcloud_to_scan.launch.py target_frame:=base_link
```

## 调参重点

最重要的是 `min_height` 和 `max_height`。它们决定从 3D 点云中截取哪一层高度来当作 2D 激光。

默认值：

```bash
min_height: -0.30
max_height: 0.30
```

如果 `/scan` 太稀疏，适当放宽高度范围；如果地面点太多或近处误报，收窄范围并提高 `range_min`。

## 注意

Humble 版 `pointcloud_to_laserscan` 会在 `/scan` 有订阅者时才订阅输入点云，所以单独播放 bag 但无人订阅 `/scan` 时，可能看起来“没有转换”。`bag_to_scan_test.launch.py` 里的 `scan_probe` 就是为了稳定触发这条链路。
