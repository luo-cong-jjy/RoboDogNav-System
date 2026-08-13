# M20 仓库巡检系统源码栈

`m20_warehouse_system` 是仿真和实机共用的顶层交付目录。它不是一个巨型 ROS 2
package；目录内保留可独立构建、测试和替换的 package 边界。ROS 包名、
节点名、话题名和现有 `ros2 launch m20_warehouse_inspection ...` 入口不因目录归并
而变化。

## 目录边界

```text
m20_warehouse_system/
├── common/
│   ├── m20_warehouse_interfaces     # 跨模块 msg/srv/action
│   └── m20_official_description    # 官方 M20 模型包装
├── navigation/
│   ├── m20_multifloor_map          # 活动地图与点云代际
│   ├── m20_scan_planner            # M20 适配的 SCAN 核心
│   └── m20_scan_navigation         # 导航网关与 SCAN bringup
├── motion/
│   └── m20_locomotion_control      # 速度适配、厂家运控后端
├── safety_mission/
│   └── m20_inspection_core         # 急停、避碰、任务与切层事务
├── simulation/
│   ├── m20_warehouse_sim           # RViz 平面运动学后端
│   ├── m20_mujoco_backend          # MuJoCo 动力学后端
│   └── m20_mujoco_rl_training      # 独立训练工程，非 ROS 运行包
└── bringup/
    └── m20_warehouse_inspection       # 唯一用户入口、地图、配置和文档
```

## 仿真与实机的差别

规划、导航网关、运动意图适配、安全、任务和 M20 模型只保留一套。
启动 profile 只替换数据源和最终执行后端：

| 边界 | RViz 仿真 | MuJoCo 仿真 | M20 实机 |
| --- | --- | --- | --- |
| 定位/点云 | PCD + 仿真 sensing | PCD + 仿真 sensing | `Elevator-LIO` + 双 RoboSense |
| 最终执行 | `m20_warehouse_sim` | SDK/ONNX + `m20_mujoco_backend` | `basic_server` 或 `direct_ros` |
| 机器狗消息 | 非必须 | `drdds` | `drdds` |

因此实机不是另一套系统：在同一公共功能栈外，主要增加
`Elevator-LIO` 和厂家通信/执行环境。`drdds` 也被 MuJoCo SDK profile 共用，所以作为
工作区级外部依赖继续放在本目录之外。

## 稳定入口

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to m20_warehouse_inspection
source install/setup.bash
ros2 launch m20_warehouse_inspection f1_scan_rviz.launch.py
```

程序运行时通过 ament 索引查找 package share，不依赖上述物理分类目录。
