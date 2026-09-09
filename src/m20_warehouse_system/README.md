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

## 运行边界

日常运行只需要关注 `bringup/m20_warehouse_inspection/launch/` 中的三个入口：

| 用途 | 入口 | 后端 |
| --- | --- | --- |
| MuJoCo 双层巡检仿真 | `inspection_mission_mujoco.launch.py` | MuJoCo + SDK/ONNX |
| 实机双层巡检 | `inspection_mission_hardware.launch.py` | Elevator-LIO + 厂家运控 |
| 仅启动导航图调试 | `inspection_mission_rviz.launch.py` | RViz 运动学后端 |

这三个入口共同复用 `m20_scan_planner`、`m20_scan_navigation`、
`m20_multifloor_map`、`m20_locomotion_control` 和 `m20_inspection_core`。
`floor_1/`、`floor_2/` 下的 PCD/JSON 是双层地图资产；切层由
`m20_floor_switch_manager` 和 `m20_warehouse_interfaces` 的 action/service 完成。

`clearance_*`、`narrow_passage_*`、`spacing_sweep_*`、`route_challenge_*`、
`phase5_regression.launch.py` 和对应配置属于回归/参数实验入口，不会被上述三个
稳定入口自动加载。它们保留是为了复现实验和运行测试，不应作为实机日常启动命令。

`simulation/m20_mujoco_rl_training` 是独立的训练工程，不是 ROS 运行依赖；它不参与
`colcon build --packages-up-to m20_warehouse_inspection`。

## 稳定入口

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to m20_warehouse_inspection
source install/setup.bash
ros2 launch m20_warehouse_inspection inspection_mission_mujoco.launch.py
```

### MuJoCo 仿真

MuJoCo 入口只启动一次公共导航、M20 运控和仿真后端。启动后先用于自由导航；它
不会因为启动而自动执行双层巡检任务。
上面的稳定入口是唯一的完整系统启动命令；不要再次重复执行同一个 launch。默认使用
`dense_four_corner_system.yaml`，需要无图形运行时在这条启动命令后追加
`use_rviz:=false use_mujoco_viewer:=false`。启动后可在 RViz 使用
`2D Goal`，或发布自由导航目标：

```bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: -35.0, y: 0.0}, orientation: {w: 1.0}}}"
```

MuJoCo 使用官方策略的 `m20_policy_v1` 能力档案：高曲率转向会保持
`0.35--0.45 m/s` 的滚动速度，不使用实机工厂档案的原地偏航。若
`/m20/sim/backend_fault` 报告 `EXCESSIVE_TILT`、`BASE_HEIGHT_LOW` 或
`NONFINITE_STATE`，后端会锁存首个故障并冻结仿真现场；请先停止当前 launch，
排查原因后重新启动以清除故障。

确认自由导航和 M20 运动适配正常后，在第二个终端启动任务触发 launch。这里不要再次
执行 `inspection_mission_mujoco.launch.py`，否则会重复启动 MuJoCo、SCAN 和运控
节点。该 launch 只连接第一终端已经启动的任务执行器，并发送当前配置中的任务 ID：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch m20_warehouse_inspection start_inspection_mujoco.launch.py \
  mission_id:=dense_four_corner_patrol
```

上面的 `dense_four_corner_patrol` 对应默认 `dense_four_corner_system.yaml`；如果
换用 `flat_multifloor_system.yaml`，任务 ID 应改为 `two_floor_warehouse_demo`。
也可以直接检查任务执行器和接口：

```bash
ros2 pkg executables m20_warehouse_inspection | grep m20_start_inspection
ros2 action list | grep /m20/mission/run
```

因此“自由导航”和“双层巡检”共享同一运行时图，但由不同 launch 触发：前者发布单个
导航目标，后者由第二终端的任务触发 launch 发送任务 Action；第一终端的 launch
本身不会自动把两步串成巡检流程。

### 实机部署

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_hardware.launch.py \
  system_config:=$(ros2 pkg prefix m20_warehouse_inspection)/share/\
m20_warehouse_inspection/config/flat_multifloor_system.yaml
```

实机启动前应先启动厂家 DDS/运控和 Elevator-LIO，并确认 `ROS_DOMAIN_ID`、传感器
话题及 TF 与 `docs/real_robot_migration/` 中的检查表一致。该入口不会启动 MuJoCo，
也不会伪造实机定位数据。

### 启动前检查

```bash
ros2 run m20_warehouse_inspection m20_validate_config --require-assets
ros2 topic list
ros2 action list | grep -E 'run_mission|switch_floor|navigate_floor'
```

程序运行时通过 ament 索引查找 package share，不依赖上述物理分类目录。
