# M20 机器狗导航与仓库巡检工作空间

本工作空间面向云深处 M20-pro 轮足机器狗，包含两条主要技术路线：

- `m20_nav2_system`： Nav2 + Gazebo 的二维导航、雷达和工厂巡检验证,仿真底层可扩展为Mujoco➕官方强化学习运控ONNX部署包，后续亦可部署到实机。
- `m20_warehouse_system`：融合FAST-Livo2的电梯扩展雷达建图定位，SCAN-Planner、多区域/多楼层任务、安全控制以及 MuJoCo/实机后端。

ROS 包、第三方依赖、数据集和工程文档按用途分开管理，构建产物不属于源码。

`m20_nav2_system` 的 Gazebo 与 MuJoCo 仿真入口严格分离：Gazebo 使用带 `_gazebo`
后缀的参数、RViz、world、URDF 和辅助脚本；MuJoCo 使用独立的
`*_mujoco` 配置/任务入口以及官方 SDK、代码雷达和 MJCF 后端。两条链路只共享
Nav2 的 ROS 接口（`/scan`、`/odom`、`/tf`、`/cmd_vel`）和外部 Nav2 算法节点。

## 目录结构

```text
robodog_nav_system/
├── src/                          # 源代码（自己开发的、二次开发、第三方包）
│   ├── m20_nav2_system/          # Nav2方案集成包
│   ├── m20_warehouse_system/     # FAST-Livo2及SCAN-PLANNER方案仓库巡检系统源码栈
│   └── third_party/              # 外部算法、驱动、消息和厂家 SDK
├── tools/                        # 工作区级雷达和网络诊断工具
├── build/                        # colcon 构建的中间编译缓存，cmake 构建目录，目标文件、obj、临时产物
├── install/                      # colcon 安装输出目录，编译后可执行文件、launch、yaml、msg、配置文件最终在这里
├── log/                          # colcon 构建日志，可清理
├── datasets/                     # rosbag、PCD 等测试数据
├── docs/                         # 跨模块实机部署与接口记录
```

## 源码清理边界

以下目录是可随时重建的本地产物，不属于源码：

```text
build/
install/
log/
**/__pycache__/
```

它们已经由 `.gitignore` 忽略。`src/`、`datasets/`、`docs/` 和第三方 SDK 不应按“看起来像旧文件”直接删除：
其中包含 Gazebo 兼容链路、实机迁移记录、官方模型/策略和回归数据。清理源码时应先建立文件用途清单，确认没有被 launch、CMake 或文档引用，再单独提交删除。

## 编译方式
1.工作空间完整普通编译：

   ```bash
   colcon build
   ```

2.限制编译时CPU线程，防止编译崩溃，例如只开2线程：

   ```bash
   colcon build --parallel-workers 2
   ```

   表示：最多两个 package 同时编译。

## 可清理目录
以下内容可通过重新构建或重新测试生成：
```text
build/
install/
log/
```

删除 `install/` 后，运行 ROS 节点前必须重新执行 `colcon build` 并重新加载，`install/setup.bash`。`datasets/`、`docs/`不属于普通构建缓存，为用户自创建数据。

---

## 功能包清点（src 目录 · 逐包说明与 GitHub 仓库地址）

> 本节为按要求的清点结果：说明文字优先取自各包 `package.xml`、包内 README 与顶层 README；仓库地址取自本地 `.git/config`、包内 README 或既有 `third_party/README.md` 记录。
> 与 20.04 工作区不同，本 `src/` 下**没有任何包配置 luo-cong-jjy 的 fork remote**，因此按约定「远端没有 fork 的则可忽略」——下方仅列出原作者 / 上游仓库地址，不再单列 foxy 适配 fork 地址。

### A. `m20_nav2_system/`（Nav2 二维导航方案栈 —— 本地自研集成，无公开 GitHub 仓库）

| 功能包 | 位置 | 简短说明 |
| --- | --- | --- |
| `m20_nav2_description` | `description/` | 官方 Deep Robotics M20 URDF 与 mesh 资源的 ROS 2 包装包 |
| `m20_nav2_system` | `navigation/` | Nav2 二维导航集成与验收的统一入口（launch / 参数 / RViz；Gazebo、MuJoCo、AOS 实机场景） |
| `m20_nav2_locomotion` | `locomotion/` | M20 运动执行层：将 Nav2 `/cmd_vel` 适配到 Gazebo 四轮模型 / MuJoCo 官方运动层 / 实机 SDK |
| `m20_nav2_backend` | `backend/` | MuJoCo 动力学后端与同源仓库碰撞世界（仅仿真，不部署实机） |

### B. `m20_warehouse_system/`（仓库巡检系统源码栈 —— 本地自研集成，无公开 GitHub 仓库）

| 功能包 | 位置 | 简短说明 |
| --- | --- | --- |
| `m20_warehouse_interfaces` | `common/` | 跨模块 msg/srv/action 类型定义（楼层切换等事务接口） |
| `m20_official_description` | `common/` | 官方 M20 URDF / mesh 的 ROS 2 包装 |
| `m20_multifloor_map` | `navigation/` | 多层活动地图与点云“代际”维护（面向多楼层仿真 / 巡检） |
| `m20_scan_planner` | `navigation/` | M20 适配的 SCAN-Planner 核心（节点、控制器、launch 与配置；自带 `vendor/` 子库：`scan_planner_msgs`、`bspline_opt`、`plan_env`、`traj_utils`、`path_searching`） |
| `m20_scan_navigation` | `navigation/` | 导航网关与 SCAN bringup（SCAN 规划与 Nav2 行为衔接） |
| `m20_locomotion_control` | `motion/` | 速度意图适配与厂家运控后端（`/JOINTS_CMD` 等运动接口） |
| `m20_inspection_core` | `safety_mission/` | 急停、避碰、任务管理与切层事务核心 |
| `m20_warehouse_sim` | `simulation/` | RViz 平面运动学仿真后端 |
| `m20_mujoco_backend` | `simulation/` | MuJoCo 动力学后端与同源仓库碰撞世界 |
| `m20_hardware_preflight` | `bringup/` | 实机运动接口的安全预检包 |
| `m20_warehouse_inspection` | `bringup/` | 集成、配置与 bringup 的唯一用户入口（mujoco / hardware / rviz 三套巡检 launch） |

### C. `third_party/`（第三方 / 外部依赖）

| 目录 | 简短说明 | 原作者 / 上游仓库 | fork（foxy 适配版） |
| --- | --- | --- | --- |
| `LeggedSkillDeploy` | 基于状态机的 Python 多策略部署框架（模仿学习 + 强化学习技能：Go1/Go2/Go2W/Duow 等）；本地附带 `robot_description/urdf` 多机型描述包（g1/go1/go2/go2w/duow/m20 等） | https://github.com/haozhang04/LeggedSkillDeploy （本地 `.git/config` 的 origin，已确认） | 无（本地未配置） |
| `Sophus` | C++ Lie 群库（基于 Eigen；ROS 包名 `sophus`），机器人 / SLAM 常用数学库 | https://github.com/strasdat/Sophus （本地无 `.git`，来源未在本地记录，按通用上游标注） | 无 |
| `deep_robotics_model` | Deep Robotics 官方机器人模型库：Lite3 / M20 / X30 / M20_Piper 的 URDF / MJCF / USD 低精度模型 | https://github.com/DeepRoboticsLab/deep_robotics_model （本地无 `.git`，地址见 `third_party/README.md`） | 无 |
| `sdk_deploy` | Deep Robotics 官方机器人控制 SDK 部署工程（支持 Lite3 / M20）：含 `drdds`（通信格式）、`lite3_transfer`（UDP-ROS2 桥）、`lite3_sdk_service`、`Lite3_sdk_deploy`、`M20_sdk_deploy` 等 ROS 包 | https://github.com/DeepRoboticsLab/sdk_deploy （本地无 `.git`，地址见 `third_party/README.md`） | 无 |
| `山猫M20 开发指南` | 云深处 M20 厂商开发文档与二次开发教程资料（本地资料目录，无仓库） | — | — |

### D. 附注

- `m20_nav2_system` / `m20_warehouse_system` 为本地自研源码栈，目录内未发现 `.git`，也未发现公开 GitHub 仓库，故不列仓库地址。
- 实机侧外部依赖（如 `Elevator-LIO`、双 RoboSense 雷达链路、工作区级 `drdds` 等）位于本工作区之外，不在本 `src/` 清单内（见顶层说明）。
- 本清单仅 `LeggedSkillDeploy` 带本地 `.git`（`origin` = haozhang04/LeggedSkillDeploy，无 fork）；`Sophus`、`deep_robotics_model`、`sdk_deploy` 均为无版本控制的副本，正式引用前建议按实际克隆来源核对 commit。
