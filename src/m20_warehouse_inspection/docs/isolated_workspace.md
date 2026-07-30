# M20 仓库巡检独立工作空间复现指南

日期：2026-07-27  
适用基线：Ubuntu 22.04 / ROS 2 Humble  
集成包：`m20_warehouse_inspection`

## 1. 目标

本指南用于从当前大工作空间中提取项目边界，并在一个全新的 ROS 2 工作空间中复现
源码、固定第三方依赖、构建、自动测试和无 GUI 运行。独立工作空间不读取当前工作空间
的 `build/`、`install/` 或 `log/`，也不依赖其中的其他业务包。

脚本复制的十个项目包是：

```text
m20_warehouse_interfaces
m20_multifloor_map
m20_official_description
m20_warehouse_sim
m20_inspection_core
m20_scan_planner
m20_scan_navigation
m20_locomotion_control
m20_mujoco_backend
m20_warehouse_inspection
```

实际 Humble 构建闭包还包含固定版本 SCAN-Planner 中的十个算法/独立演示依赖包，
以及固定 M20 SDK 仓库中的两个包：

```text
local_sensing_node
map_generator
mockamap
plan_env
pose_utils
scan_planner_msgs
path_searching
bspline_opt
traj_utils
odom_visualization
drdds
m20_sdk_deploy
```

因此 `--packages-up-to m20_warehouse_inspection` 的预期闭包为 22 个包。后四个附加
依赖来自 `m20_scan_planner` 保留的独立 Mockamap/里程计显示入口，不会进入仓库
bringup 的运行时节点图。`drdds` 和 `m20_sdk_deploy` 只在完整 MuJoCo/SDK profile
进入运行时节点图。官方模型仓库中与
本项目无依赖关系的其他 description/工具包不会进入该闭包。

## 2. 依赖锁

版本真值位于：

```text
config/workspace_lock.yaml
config/rviz_v1_baseline.yaml
dependencies.repos
dependencies_sdk.repos
patches/scan_planner_humble_cpu.patch
patches/scan_planner_multifloor_integration.patch
patches/m20_sdk_cmdvel_adapter.patch
```

当前固定内容：

```text
Project release
  version   1.3.0

SCAN-Planner
  revision  d0b921c9b05a6d291d144d60882b2e0e88d2c0e0

Deep Robotics model
  revision  6113c62da96295e8d53abbc079af5296bf4649f8

M20 SDK
  revision  ee289d475f2dedf0332b7542f2b173fa9e8d1456

SCAN Humble CPU patch
  sha256    8cfa3e13bfd3c7ddd0311eeaf46058ffbe61cd28e75dff633968e53cf17de974

SCAN multi-floor integration patch
  sha256    e5080a336087e78e74036446b16b24f1ffd47e1021396bad501f2a0f5778e4a4

M20 SDK cmd_vel adapter patch
  sha256    3e97c2554af7594470d928107951a42abf60a6db3453cc12cc2122b5a771c633
```

准备脚本会先检查三个 Git revision 和三个 patch SHA-256，再依次应用补丁；任一项
不一致都会失败。CPU 补丁让 CPU profile 不再解析或声明仅 GPU renderer 才需要的
GLM；集成补丁固化当前已验证的 SCAN 规划适配、楼层 reset/地图重载和原生局部点云
传输接缝；SDK 补丁固化 `/m20/locomotion/cmd_vel_sdk` 到官方策略状态机的 ROS
入口。`rviz_v1_baseline.yaml` 另外锁定 1.3.0 稳定配置、PCD/PGM 和官方
M20 URDF；实验性的 `route_challenge` 不属于冻结哈希。如以后启用 `USE_GPU`，
应改用单独的 GPU 依赖 profile。

## 3. 在线准备

从当前源码树执行：

```bash
src/m20_warehouse_inspection/tools/prepare_isolated_workspace.sh \
  /path/to/m20_warehouse_ws
```

目标目录必须不存在或为空。脚本不会删除、覆盖已有工作空间。在线模式使用
`dependencies.repos` 通过 `vcs import` 获取固定 revision。

然后安装系统依赖：

```bash
source /opt/ros/humble/setup.bash
cd /path/to/m20_warehouse_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --user "mujoco==3.10.0"
```

`rosdep update` 只需要在本机索引陈旧或缺失时执行。

若 `rosdep check` 同时报告 `eigen`、`python3-yaml`、`python3-numpy` 等常见键没有
definition，应先检查 `/etc/ros/rosdep/sources.list.d/20-default.list`。`base.yaml`、
`python.yaml` 和 `ruby.yaml` 行末不应附加 `base`、`python`、`ruby` 过滤标签；这是
本机源配置问题，不应通过改写项目的标准 `package.xml` 依赖名来绕过。

## 4. 离线或本地镜像准备

如果机器上已有三个完整 Git 仓库，可避免网络访问：

```bash
src/m20_warehouse_inspection/tools/prepare_isolated_workspace.sh \
  /path/to/m20_warehouse_ws \
  --scan-repository /path/to/SCAN-Planner \
  --model-repository /path/to/deep_robotics_model \
  --sdk-repository /path/to/sdk_deploy
```

本地仓库仍必须含有锁文件指定的 commit。脚本使用独立 clone，不会修改输入仓库。

成功后，工作空间根目录会生成 `m20_workspace.lock`，记录实际 checkout revision 和
patch 哈希，便于归档构建产物时追踪源码来源。

## 5. 严格隔离构建

为排除当前终端曾 source 其他工作空间的影响，先清理 overlay 变量，只加载系统 ROS：

```bash
cd /path/to/m20_warehouse_ws
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash

colcon list --topological-order --packages-up-to m20_warehouse_inspection
colcon build --symlink-install \
  --packages-up-to m20_warehouse_inspection \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

构建前 `AMENT_PREFIX_PATH` 应只含 `/opt/ros/humble`。构建完成后再加载本工作空间：

```bash
source install/setup.bash
```

## 6. 自动测试与静态入口检查

```bash
colcon test --packages-select \
  m20_warehouse_interfaces \
  m20_multifloor_map \
  m20_official_description \
  m20_warehouse_sim \
  m20_inspection_core \
  m20_scan_planner \
  m20_scan_navigation \
  m20_locomotion_control \
  m20_mujoco_backend \
  m20_warehouse_inspection \
  --return-code-on-test-failure

colcon test-result --verbose

ros2 run m20_warehouse_inspection m20_validate_config --require-assets
ros2 interface show m20_warehouse_interfaces/action/SwitchFloor
ros2 launch m20_warehouse_inspection phase5_regression.launch.py --show-args
```

注意目录名 `m20_inspection_core` 和 `m20_multifloor_map` 就是实际 ROS 包名；测试选择应
使用包名而不是自行推测的名称。

## 7. 无 GUI 回归

本机需要足够的 DDS participant 资源。项目提供 participant 容量配置；接口由
CycloneDDS 自动选择，并用独立 domain 隔离：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=79
export CYCLONEDDS_URI=file://$(ros2 pkg prefix \
  m20_warehouse_inspection)/share/m20_warehouse_inspection/config/cyclonedds_local.xml
```

三类回归入口：

```bash
# F1/F2 交替切换 20 次
ros2 launch m20_warehouse_inspection phase5_regression.launch.py \
  mode:=switch_stress switch_iterations:=20 use_rviz:=false

# 历史稳定任务 10 次
ros2 launch m20_warehouse_inspection phase5_regression.launch.py \
  mode:=mission_stress mission_runs:=10 use_rviz:=false

# 错误请求、旧代次、取消、停止和故障重试
ros2 launch m20_warehouse_inspection phase5_regression.launch.py \
  mode:=fault_injection use_rviz:=false
```

回归进程退出后，第 5 阶段专用 launch 会向整个 bringup 图发送有序关闭事件。自动化
环境除检查进程退出外，还应检查对应 `PHASE5_*_PASS` 标记，区分通过与故障退出。

路线挑战场景应先完成单次 11 步运行，再决定是否执行多轮回归：

```bash
ros2 launch m20_warehouse_inspection \
  route_challenge_mission_rviz.launch.py use_rviz:=false

ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id route_challenge_four_corner_patrol
```

它不继承冻结配置的历史 10/10 结果。

## 8. 实机迁移边界

独立工作空间同时冻结纯 RViz profile 和 MuJoCo+官方 SDK 完整动力学 profile，但
仿真结果不代表可以直接部署到 M20。实机 profile 必须替换：

- `/m20/sim/body_pose` 的 MuJoCo 后端；
- PCD 裁剪 local sensing；
- `/JOINTS_CMD`、`/JOINTS_DATA`、`/IMU_DATA` 的仿真传输为真实硬件接口；
- 楼层切换后的真实定位重初始化与质量确认。

`RunMission`、`NavigateFloor`、`SwitchFloor`、floor/generation、fail-closed 安全
输出，以及已验证的 SDK 速度入口保持不变，这正是独立拆包需要保护的迁移边界。
