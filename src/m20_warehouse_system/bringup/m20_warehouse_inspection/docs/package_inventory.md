# M20 仓库巡检集成包清单

## 交付形态

项目自有源码统一位于 `src/m20_warehouse_system/`，用户只使用 ROS 包
`m20_warehouse_inspection` 作为系统入口。它拥有系统 YAML、地图、RViz、
launch、验收工具和开发文档；一次 `--packages-up-to m20_warehouse_inspection` 会构建
完整闭包。ROS 2 内部仍保留必要的模块包边界，避免消息生成、C++ 规划库、官方模型和
不同许可证第三方代码相互循环依赖。

顶层分类为 `common/`、`navigation/`、`motion/`、`safety_mission/`、
`simulation/` 和 `bringup/`。详细树形结构见
[`../../../README.md`](../../../README.md)。

## 十个业务模块包与一个接口模式包

| 包 | 职责 |
| --- | --- |
| `m20_warehouse_inspection` | `bringup/`；唯一用户入口、总配置、地图资产、launch 与验收 |
| `m20_warehouse_interfaces` | `common/`；Action、msg、srv；不包含业务实现 |
| `m20_official_description` | `common/`；云深处官方 M20 URDF、mesh 和 wrapper |
| `m20_multifloor_map` | `navigation/`；active floor/generation 对应的局部点云数据面 |
| `m20_inspection_core` | `safety_mission/`；任务、切层事务、安全监督和碰撞保护 |
| `m20_locomotion_control` | `motion/`；安全速度到 M20 SDK/厂家运控的适配 |
| `m20_warehouse_sim` | `simulation/`；RViz 平面运动学后端 |
| `m20_mujoco_backend` | `simulation/`；官方 SDK/ONNX 与 MuJoCo 动力学后端 |
| `m20_scan_planner` | `navigation/`；适配后的 SCAN 节点、B 样条控制与 reset |
| `m20_scan_navigation` | `navigation/`；typed 导航网关、点云链路与 SCAN bringup |
| `drdds` | 唯一启用的 M20 DrDDS 消息/服务模式，兼容高层运动与低层 SDK |

## 十一个第三方构建依赖包

`bspline_opt`、`local_sensing_node`、`map_generator`、`mockamap`、
`odom_visualization`、`path_searching`、`plan_env`、`pose_utils`、
`scan_planner_msgs`、`traj_utils`、`m20_sdk_deploy`。

这些包来自锁定 revision 的 SCAN-Planner 与云深处 SDK。它们属于算法/SDK 依赖，不是
用户启动入口，也不应复制进集成包形成源码分叉。

## Foxy 实机扩展包

背部 x86/Foxy 实机闭包在上述公共功能栈之外增加 `lio`（目录
`src/Elevator-LIO`），负责双 RoboSense + `/IMU` 的连续定位、去畸变世界系点云和
电梯状态估计。仿真闭包不依赖它，避免把 PCL/雷达驱动和 MuJoCo 回归强行绑定。

工作区顶层 `src/drdds` 是唯一启用的完整接口包，合并 SDK 低层关节/IMU/电池类型
和完整开发指南给出的 `NavCmd/MotionInfo/MotionState/Gait`。SDK 随附的旧同名副本由其
自身 `COLCON_IGNORE` 隔离。实机可用 `factory_transport` 在 `basic_server` 与
`direct_ros` 间选择，且同一时刻只能启用一个。

`drdds` 不是纯实机包：MuJoCo + 官方 SDK/ONNX profile 也使用低层关节和
IMU 接口。因此它作为仿真/实机共享的工作区级依赖，保持与顶层系统目录
并列。

## 默认隔离的源包

以下包不属于当前 22 包闭包，目录内放置 `COLCON_IGNORE`：

- `m20_foxy_nav_deploy`：旧 Foxy 部署实验。
- `m20_industrial_inspection*`：早期 Gazebo/MuJoCo 集成原型。
- `m20_nav2_gazebo_sandbox`：非主线 Nav2/Gazebo 沙箱。
- `m20_motion_demo`：独立运动演示。
- `m20_lidar_bridge`、`m20_lidar_to_scan`：实机雷达实验代码，保留源码但默认不构建。

实机阶段如需恢复雷达实验包，应先迁移其接口到当前 typed sensing contract，随后删除
对应 `COLCON_IGNORE` 并单独构建验证，不能让旧节点直接加入生产启动图。

## 标准命令

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to m20_warehouse_inspection
source install/setup.bash
ros2 launch m20_warehouse_inspection f1_scan_rviz.launch.py
```

因此项目在操作层面是“一个集成功能包、一条启动命令”，在实现层面保持可测试、可替换
和可独立移植的模块化闭包。

面向实物部署的逐包目录、运行职责、数据链以及保留/替换关系见
[`real_robot_migration/01_simulation_module_inventory.md`](real_robot_migration/01_simulation_module_inventory.md)。
