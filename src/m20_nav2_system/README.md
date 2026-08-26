# M20 Nav2 System

本包保留两条可选仿真链路：

- Gazebo：`factory_navigation.launch.py`，现有四轮模型、雷达和 Nav2 链路保持不变。
- MuJoCo：`m20_mujoco_navigation.launch.py`，复用 `m20_warehouse_inspection` 的官方
  M20 MJCF/ONNX 运动层、MuJoCo odom/TF，以及基于 PCD 的 CPU ray-casting 感知。

MuJoCo 后端接入当前二维 Nav2 的桥接入口为
`m20_mujoco_sensor_bridge.launch.py`：它把 `/m20/sim/body_pose` 适配为 `/odom` 和
`odom -> base_link` TF，并根据 `/map` 栅格生成标准 `/scan`。该桥接不参与 Gazebo 默认
启动链路，Gazebo 仍使用自身的雷达和里程计插件。
雷达固定坐标系为 `base_scan`，桥接入口同时发布 `base_link -> base_scan` 静态 TF。

推荐默认启动命令已经包含 MuJoCo viewer 和 RViz，无需追加参数：

```bash
ros2 launch m20_nav2_system m20_mujoco_navigation.launch.py
```

MuJoCo 入口不会复制官方策略或修改 Gazebo 模型。使用前需先构建并安装
`m20_warehouse_inspection` 及其 `m20_mujoco_backend` 依赖；`system_config` 应指向该包
使用的 system YAML。两条入口互不替换，默认的 Gazebo 调试流程仍按下文命令运行。

桥接的独立检查命令（需在已 source ROS 2 和工作空间的终端执行）：

```bash
ros2 launch m20_nav2_system m20_mujoco_sensor_bridge.launch.py
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

工厂静态碰撞场景生成与校验：

```bash
ros2 launch m20_nav2_system generate_factory_mujoco_world.launch.py
```

该命令从 `worlds/factory_environment.world` 提取地面、围墙和 5 排工作台，注入官方
M20 MJCF，并检查 16 个执行器、浮动基座、IMU 与静态几何数量。Gazebo world 中的动态
worker 代理及其 waypoint 插件尚未迁移到 MuJoCo；在动态障碍物迁移完成前，MuJoCo 入口
只用于静态工厂场景和运动层验证。

MuJoCo backend 也支持在启动时直接从 Gazebo SDF 生成场景：

```bash
ros2 launch m20_nav2_system m20_mujoco_navigation.launch.py \
  world_source:=factory_sdf \
  world_file:=/path/to/m20_nav2_system/worlds/factory_environment.world
```

该模式会在 backend launch 内生成并加载官方 M20 MJCF。当前仅转换静态 box/cylinder
碰撞体；Gazebo world 中由 waypoint 插件驱动的动态 worker 仍未迁移。
转换器会在终端明确打印被跳过的动态模型名称，避免将静态场景结果误认为完整动态场景。

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
