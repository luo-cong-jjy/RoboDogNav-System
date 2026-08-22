# M20 Nav2 System

ROS 2 Humble / Gazebo Classic 集成包，提供 M20 自由导航仿真、独立巡检任务和 PCD 点云地图验证。

## 正式入口

| 模块 | 命令 | 说明 |
| --- | --- | --- |
| 自由导航仿真 | `ros2 launch m20_nav2_system factory_navigation.launch.py` | Gazebo、机器人、雷达、Nav2、RViz |
| 巡检任务 | `ros2 launch m20_nav2_system factory_inspection_mission.launch.py` | 向已运行的 Nav2 发送巡检目标 |
| PCD 验证 | `ros2 launch m20_nav2_system pcd_validation.launch.py` | PCD 切片、栅格化和 costmap 验证 |

巡检任务不会由仿真入口自动启动，必须在自由导航确认正常后单独启动。

## 目录结构

```text
m20_nav2_system/
├── launch/       # 正式仿真、巡检、Nav2、雷达和 PCD 入口
├── config/       # Nav2、雷达、巡检和 PCD 参数
├── maps/
│   ├── factory/  # 仿真/Nav2 默认工厂栅格地图
│   └── pcd/
│       ├── raw/  # 原始 PCD；默认使用 t100ipro_2026-07-14-11-56-57.pcd
│       └── grids/# PCD 生成的 PGM/YAML 与 A* 输出
├── models/       # Gazebo URDF、可视化 URDF 和网格
├── rviz/         # RViz 配置
├── scripts/      # 正式运行节点
│   └── pcd/      # PCD 专用工具
├── src/          # Gazebo 插件和 C++ 测试节点
├── worlds/       # Gazebo 世界
└── docs/         # 接口、设计和 PCD 文档
```

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/robodog_nav_system
colcon build --packages-select m20_nav2_system --symlink-install
source install/setup.bash
```

需要时可指定日志目录：

```bash
export ROS_LOG_DIR=/tmp/m20_ros_log
mkdir -p "$ROS_LOG_DIR"
```

## 自由导航仿真

默认使用轻量 2D 雷达，Gazebo 直接发布 `/scan`：

```bash
ros2 launch m20_nav2_system factory_navigation.launch.py
```

启动顺序为 Gazebo/机器人/雷达、Nav2、RViz。雷达模式：

Gazebo GUI 默认关闭，以避免 WSL/远程 OpenGL 环境出现黑屏；需要查看 Gazebo 窗口时显式设置
`use_gazebo_gui:=true`。

```bash
# 默认 2D
ros2 launch m20_nav2_system factory_navigation.launch.py lidar_mode:=2d

# 3D PointCloud2，经转换节点生成 /scan
ros2 launch m20_nav2_system factory_navigation.launch.py lidar_mode:=3d
```

常用参数：

```bash
ros2 launch m20_nav2_system factory_navigation.launch.py \
  launch_rviz:=true spawn_delay:=6.0 nav2_delay:=10.0 rviz_delay:=20.0
```

检查导航是否就绪：

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 action list | grep navigate
```

## 巡检任务

确认 Nav2 已进入 `active` 后，另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/robodog_nav_system/install/setup.bash
ros2 launch m20_nav2_system factory_inspection_mission.launch.py
```

该入口只启动巡检 action 客户端，不会重复启动 Gazebo、Nav2 或 RViz。巡检点位配置位于：

```text
config/factory_inspection_midpoints.yaml
```

## PCD 地图验证

PCD 工具统一位于 `scripts/pcd/`：

- `pcd_to_occupancy_grid`：将 PCD 投影为 PGM/YAML 栅格地图。
- `pcd_slice_publisher`：对大型 PCD 做高度/距离切片并发布 PointCloud2。
- `grid_astar_planner`：离线检查栅格连通性和 A* 路径。
- `lifecycle_configure_activate`：PCD 验证场景的 lifecycle 辅助节点。

离线转换示例：

```bash
python3 src/m20_nav2_system/scripts/pcd/pcd_to_occupancy_grid
```

不带参数时使用 `maps/pcd/raw/t100ipro_2026-07-14-11-56-57.pcd`，并在
`maps/pcd/grids/` 自动生成包含时间戳、高度范围、距离范围和分辨率的文件名。也可以显式指定输入和输出：

```bash
python3 src/m20_nav2_system/scripts/pcd/pcd_to_occupancy_grid \
  --pcd /path/to/map.pcd --output /tmp/m20_map
```

启动验证：

```bash
ros2 launch m20_nav2_system pcd_validation.launch.py
ros2 launch m20_nav2_system pcd_to_nav2_costmap.launch.py
ros2 launch m20_nav2_system pcd_nav2_planner.launch.py
```

指定点云：

```bash
ros2 launch m20_nav2_system pcd_validation.launch.py pcd_path:=/path/to/map.pcd
```

## 接口约定

```text
2D lidar                         -> /scan
3D lidar                         -> /LIDAR/POINTS -> /scan
Gazebo drive plugin              -> /odom, /tf
Nav2 controller                  -> /cmd_vel
TF chain                         -> map -> odom -> base_link -> lidar_link
```

真机接入时，Nav2 输出应先经过限速、急停、命令超时和人工接管等安全层，再连接运动 SDK。

## 故障排查

找不到包或启动旧代码：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select m20_nav2_system --symlink-install
source install/setup.bash
```

RViz 不显示机器人或雷达：

```bash
ros2 topic info /scan -v
ros2 topic echo /tf --once
ros2 topic echo /robot_description --once
```

确认 `/scan` 使用兼容的 `Best Effort` QoS，并确认 TF 链完整。Nav2 action 不存在时，检查
`/bt_navigator`、`/planner_server` 和 `/controller_server` 是否均为 `active`。

不同项目需要通信时使用相同 `ROS_DOMAIN_ID`；需要隔离仿真时使用不同 domain，避免同时启动
两套同名 Nav2 容器或两套巡检任务。
