# 实物部署准备 03：Foxy x86、drdds 与 Elevator-LIO 集成

> 基线日期：2026-08-12  
> 目标主机：M20 背部 x86，Ubuntu 20.04，ROS 2 Foxy  
> 当前状态：Humble 源码/仿真回归环境中完成适配；Foxy 目标机原生构建和实机运动尚未验收

本机验证边界：`Elevator-LIO` 只做源码、话题、坐标系和启动链静态审查，不纳入当前
Humble 仿真工作空间的编译闭包。它只应在背部 Foxy x86 主机上连同实际 RoboSense、
IMU 驱动构建和验证；当前主机继续只回归导航、任务、安全和 MuJoCo 仿真链。

## 1. 最终结论

目标架构可以采用背部 Foxy x86 单机部署，SCAN、巡检、安全、Elevator-LIO 和出厂运动
适配器都在该主机运行。Humble 的 `build/`、`install/` 不能复制过去，必须在 Foxy 主机
从同一源码重新构建。

出厂运动层的选择分为两个层次：

- 系统内部始终使用统一的安全速度契约
  `/m20/locomotion/cmd_vel_sdk`（`geometry_msgs/Twist`）；
- 最后一跳可选 `basic_server` 或 direct ROS `/NAV_CMD`。这只是传输方式切换，不影响
  SCAN、任务和安全层。

两种后端现均已完成源码实现。默认继续使用 `basic_server`，因为完整开发指南明确建议其
优先用于运动控制，内置状态机、调用更简单；`direct_ros` 适合本项目这种机载自定义导航
算法，并保留为一条完整可选链。无论选择哪种，实机首次放行前都必须核对固件版本、实际
QoS、控制权和急停行为。

## 2. 两种运动后端与速度反馈

### 2.1 已确认的源码事实

完整开发指南补齐了以下官方结构：

1. `/NAV_CMD`：`NavCmd`，`x_vel/y_vel/yaw_vel`；
2. `/MOTION_INFO`：`MotionInfo`，20 Hz，包含实测 `vel_x/vel_y/vel_yaw`、状态和步态；
3. `/MOTION_STATE`：`MotionState`，显式切换站立、RL、趴下和软急停；
4. `/GAIT`：`Gait`，停稳后切换步态；
5. `MetaType` 使用 `builtin_interfaces/Time stamp`；`MotionInfoValue` 的
   `state/gait` 是扁平字段；`/HES_STATUS` 使用 `StdMsgInt32.value`。

顶层 `src/drdds` 已成为唯一启用的完整消息包，同时保留 SDK 使用的低层关节、IMU、
电池类型。`src/drdds-背部主机当前版` 是 M20-PRO 已部署 v1.2.0 基准，仅供 ABI 比对并用
`COLCON_IGNORE` 隔离；SDK 内旧同名包也被隔离。静态预检会比较全部 25 个消息的规范化
字段，而不是按旧手册截图推断。

当前预检结果为：

```text
Foxy source preflight: PASS
selected factory transport: direct_ros
drdds interface schema: MATCHES BACKPACK BASELINE; direct_ros QoS: TARGET TEST PENDING
```

### 2.2 速度反馈选择

`/MOTION_INFO` 是 direct ROS 模式下最合适的速度反馈。其三个速度字段是机体系实际运动
状态，单位和正负方向与 `/NAV_CMD` 一致，不能用 `/LIO/odom_vehicle` 的空 twist 替代。
direct 后端把它转换为统一 `/m20/locomotion/measured_twist`；basic_server 后端则把 TCP
`MotionStatus` 转成同一个话题。定位适配器和导航速度闭环不感知后端差异。

### 2.3 最终选择

- **默认 `basic_server`**：符合厂家“优先用于运动控制”的建议，自动推进运动状态，适合
  首次台架、诊断和保守生产回退。
- **可选 `direct_ros`**：少一层 TCP/UDP JSON 转换，直接获得 `/MOTION_INFO`，更适合背部
  Foxy 主机运行自定义导航；但必须由本后端显式完成 `1 -> 17 -> gait` 状态机。
- 两者不能同时启动。共同的 `command_ownership_confirmed=false`、手动 enable、300 ms
  本地超时、500 ms 固件看门狗边界和安全零速策略不变。

目标机仍需执行：

```bash
ros2 interface show drdds/msg/NavCmd
ros2 interface show drdds/msg/MotionInfo
ros2 interface show drdds/msg/MotionState
ros2 interface show drdds/msg/Gait
ros2 interface show drdds/msg/StdMsgInt32
ros2 topic info /NAV_CMD --verbose
ros2 topic info /MOTION_INFO --verbose
ros2 pkg prefix drdds
```

并验证实际 QoS、20 Hz 反馈、20 Hz 命令、500 ms 看门狗、导航使用模式、步态状态、机载
planner/自动充电冲突和遥控接管。手册要求 M20 使用 Fast DDS；目标机设置
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、与机器狗一致的 `ROS_DOMAIN_ID`（文档示例为 0）。

## 3. 仿真与真机启动链的准确差异

### 3.1 共同链路

```text
RViz 手动目标 / 巡检任务
  -> 地图与楼层 generation
  -> SCAN-Planner + 可选栅格路线
  -> closed_loop_controller
  -> navigation_adapter
  -> collision_guard + safety_supervisor
  -> locomotion_manager
  -> /m20/locomotion/cmd_vel_sdk
```

目标、任务、规划、安全和楼层事务都保留；差异只在感知/定位输入和最终执行后端。

### 3.2 Humble + MuJoCo 仿真

```text
仿真 PCD -> local_sensing ray caster -> /quad_0/cloud
MuJoCo -> /m20/sim/body_pose + IMU + joint_states

/m20/locomotion/cmd_vel_sdk
  -> 本地 M20 ONNX/RL 策略
  -> /JOINTS_CMD
  -> MuJoCo 16 执行器
  -> 仿真位姿/速度反馈
```

启动入口：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
ros2 launch m20_warehouse_inspection inspection_mission_mujoco.launch.py
```

### 3.3 Foxy + M20 实机

```text
/rslidar_points_front + /rslidar_points_rear + /IMU
  -> Elevator-LIO
  -> /LIO/clouds_lidar       (去畸变、world 系点云)
  -> /LIO/odom_vehicle       (机体位姿)
  -> /LIO/odom_imu           (传感器位姿/速度)
  -> /LIO/in_elevator + /LIO/elevator_state

/LIO/odom_vehicle + factory measured Twist
  -> m20_hardware_localization_adapter
  -> /m20/localization/body_pose (位姿 + 可用的机体系速度)

/m20/locomotion/cmd_vel_sdk
  -> basic_server 或 direct_ros（启动参数二选一）
  -> AOS 出厂运动控制器
  -> 真实轮腿
  -> MotionStatus 或 /MOTION_INFO
  -> /m20/locomotion/measured_twist
```

真机不启动 MuJoCo、本地 ONNX 关节策略、PCD ray caster、`/JOINTS_CMD` 仿真执行器或
RViz 运动学位姿积分器。一个启动图中只能有一个 `world -> base_link` 权威发布源和一个
最终运动后端。

## 4. Elevator-LIO 接入结果

### 4.1 已使用的真实接口

本地源码确认：

- `/LIO/clouds_lidar` 在发布前调用 `transLidar2World`，消息内容已经是 world 坐标，
  可直接作为 SCAN 实时点云，不能再按原始雷达坐标重复变换；
- `/LIO/odom_vehicle` 提供 body 位姿与 `world -> body` TF，但当前未填 Odometry twist；
- `/LIO/odom_imu` 提供 IMU 位姿和线速度；
- `/LIO/in_elevator` 为锁存布尔状态；
- `/LIO/elevator_state` 提供相对位移、速度、加速度；
- `/LIO/set_elevator_flag` 可手动进入/退出电梯模式。

新增 `root_config_m20_navigation.yaml`，保持已验证的双 RoboSense 话题和外参，只把输出
坐标系收敛为系统使用的 `world`、`base_link`、`m20_lio_imu`。原来的
`root_config_m20.yaml/robosense_m20.yaml` 未改，可独立继续对照测试。

### 4.2 为什么增加定位适配器

SCAN 主要使用位姿，但栅格分段停车、楼层切换停止确认也读取 Odometry twist。
`/LIO/odom_vehicle` 当前 twist 默认为零，直接使用会在机器狗仍运动时误判“已经停住”。

新增适配器做以下工作：

1. 校验输入必须是 `world -> base_link`，不匹配则不发布、ready=false；
2. 优先把出厂 `MotionStatus` 或 `/MOTION_INFO` 的机体系速度合并到 LIO 位姿；
3. 状态短暂缺失时，由连续 LIO 位姿差分得到有界机体系速度；
4. 发布统一 `/m20/localization/body_pose` 和可诊断的 adapter state。

它不修改 LIO 算法、不复制高带宽点云，也不把运动速度反馈伪装成全局定位。

### 4.3 坐标与地图约束

真实每层 PCD/占据地图、巡检点和 LIO `world` 必须使用同一原点与朝向。建图模式默认从
启动点建立原点；重定位模式只能加载一个指定 PCD，且 README 明确说明需在地图原点附近
启动。当前 Elevator-LIO 没有动态“切 PCD + 设置任意初始位姿”的 ROS 服务。

因此现有仿真的 `/m20/localization/set_pose` 不能直接映射为 Elevator-LIO 功能。真实楼层
profile 应使用 `pose_handoff: wait_for_target` 或 `none`，由外部运输/定位 provider 确认，
不能用仿真传送。

## 5. 多层/电梯能力边界

Elevator-LIO 解决的是电梯运动期间 LIO 退化和状态估计，不负责：

- 呼梯、选层、门状态协议；
- 判断目标楼层编号；
- 自主驶入/驶出轿厢的运动所有权；
- 每层地图选择和任意初始位姿重定位。

系统已有参数化边界仍然有效：

- `transfer_adapter: timed_hold` 仅仿真；
- `transfer_adapter: external_action` 用于实机 provider；
- `pose_handoff: preserve/wait_for_target/none/set_simulation_pose` 独立配置；
- 地图提交、generation、导航复位和点云 fresh 检查与具体电梯解耦。

第一阶段实机建议用 `external_action + supervised_manual_transport + wait_for_target`：系统
保持零速，由人工/遥控完成运输，provider 监测 Elevator-LIO 和楼层信号后报告完成。
全自主驶入/驶出需要另加受安全仲裁约束的独占运动阶段，当前不能宣称已完成。

## 6. Foxy 与 Humble 的必要改动

### 6.1 已修正的源码差异

- Foxy 使用 Python 3.8，部分新增的 `dict[str, ...]`、`tuple[...]`、`A | None` 注解会在
  导入时求值；相关运行模块已加 `from __future__ import annotations`；
- 实机闭包中的 `rclpy.try_shutdown()` 已换成 Foxy/Humble 共用的
  `if rclpy.ok(): rclpy.shutdown()`；
- 新增启动文件和适配器避免 Python 3.9/3.10 专有语法；
- Elevator-LIO 的 CMake 已具有 ROS 2 分支和老版 `rosidl_target_interfaces` fallback；
- 建议 Foxy/RoboSense 目标构建关闭非必要组件：
  `-DLIO_BUILD_SIM=OFF -DLIO_BUILD_RVIZ_PLUGIN=OFF -DLIO_WITH_LIVOX=OFF`。

### 6.2 只在目标机执行的验证

- Ubuntu 20.04/GCC 9 下完整重编译 SCAN、PCL、Eigen、Elevator-LIO；当前 Humble 主机
  不编译 `Elevator-LIO`，也不以本机编译结果代替目标机验收；
- Foxy `rclpy` Action 的 goal/cancel/result 时序；
- Foxy 的 `sensor_msgs_py`、launch event、TF CLI 和 QoS 行为；
- RoboSense SDK、双雷达组播、IMU 时间戳和 PTP/系统时钟；
- 目标 RMW、`ROS_DOMAIN_ID` 与 AOS DDS 可见性；
- basic_server 到 AOS 的网络路由、状态频率、实际停车时间；
- direct ROS 若启用时的自定义类型与 QoS 完全匹配。

Foxy 已结束官方支持，所以这里的“兼容”必须以目标机干净构建和运行记录为准，不能用
Humble 编译成功代替。

## 7. 实机启动入口与保护

在 Foxy 主机完成原生构建后，单命令入口为：

```bash
source /opt/ros/foxy/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI

ros2 launch m20_warehouse_inspection \
  inspection_mission_hardware.launch.py \
  system_config:=/ABSOLUTE/PATH/TO/REAL_SITE.yaml
```

该入口默认：

- 启动 `Elevator-LIO`，默认使用
  `root_config_m20_navigation_relocation.yaml` 加载已验收地图；mapping profile 只在建图
  阶段单独启动；
- 启动定位/速度融合适配器；
- 关闭仿真 ray caster 和仿真运动后端；
- 启动 SCAN、任务、安全与 `basic_server`；
- `command_ownership_confirmed=false`、`auto_enable_motion=false`，机器人不能自动动起来。

使用 direct ROS 时只增加：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_hardware.launch.py \
  factory_transport:=direct_ros \
  system_config:=/ABSOLUTE/PATH/TO/REAL_SITE.yaml
```

direct 后端发布 `/MOTION_STATE`、`/GAIT`、`/NAV_CMD`，订阅 `/MOTION_INFO`；不会同时启动
basic_server socket 后端。

如果雷达/LIO 由 systemd 或另一终端管理：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_hardware.launch.py \
  start_elevator_lio:=false \
  system_config:=/ABSOLUTE/PATH/TO/REAL_SITE.yaml
```

预检：

```bash
python3 src/m20_warehouse_system/bringup/m20_warehouse_inspection/tools/validate_foxy_hardware_source.py \
  --transport basic_server

python3 src/m20_warehouse_system/bringup/m20_warehouse_inspection/tools/validate_foxy_hardware_source.py \
  --transport direct_ros
```

在没有实场地图和台架放行前，不得把仿真 `dense_four_corner_system.yaml` 当成实机现场
profile，也不得把 `command_ownership_confirmed` 与 `auto_enable_motion` 默认改成 true。

## 8. 仍需现场补充的信息

1. 目标机上 `drdds/msg` 的 `ros2 interface show` 与当前源码逐字段对照，以及
   `/NAV_CMD`、`/MOTION_INFO` 的实际 QoS；
2. AOS/运动固件/basic_server 版本及 Foxy 主机到 `10.21.31.103` 的网络配置；
3. Foxy 主机上 `ros2 topic list/type/hz` 与 TF 树记录；
4. 一段双雷达、`/IMU`、LIO 输出、`/MOTION_INFO` 的静止/直行/转弯 rosbag；
5. 实场每层地图、world 原点、lobby/cabin/巡检点坐标；
6. 电梯控制协议、楼层身份来源和人工/自主驶入驶出的安全规则。

缺少第 1 项不阻塞 `basic_server`，但阻塞 direct ROS 实机放行；缺少第 4～6 项则阻塞真实
闭环、多层和自主电梯验收。

逐条可执行的部署、构建、安全放行、F1 单层验收和回退流程见
[`04_m20_pao_step_by_step_deployment.md`](04_m20_pao_step_by_step_deployment.md)。
