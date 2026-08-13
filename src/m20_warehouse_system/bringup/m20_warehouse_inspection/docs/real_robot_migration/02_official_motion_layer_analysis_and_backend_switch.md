# 实物部署准备 02：官方运动层分析与仿真/真机后端切换

> 基线日期：2026-08-12  
> 结论状态：协议与软件边界已实现、MuJoCo 仿真回归通过；尚未经过实机台架放行  
> 手册来源：`src/third_party/山猫M20 开发指南/` 与早期技术手册

## 1. 结论

当前 MuJoCo 底层与 M20 出厂运动层**不能在关节层直接平替**，但可以在系统现有的
`geometry_msgs/Twist` 安全速度边界进行后端替换。

- 当前完整动力学仿真由本地 ONNX 策略根据机体速度生成 16 路腿/轮关节指令，再由
  MuJoCo 执行动力学。
- 实机推荐由本系统发送机体 `X/Y/Yaw`，让机器狗 AOS 内的出厂运动控制器协调轮子、
  四肢、站立和步态；本系统不应再同时运行本地 ONNX 关节策略。
- 因此真机替换范围是“本地 ONNX + `/JOINTS_CMD` + MuJoCo”整个执行后端，而不是只把
  MuJoCo 换成真实关节。
- SCAN、导航网关、碰撞保护、安全仲裁、巡检任务和切层事务可保留；真实定位、TF、
  雷达点云、时间同步、硬件急停和电梯接口仍必须另行接入。

这意味着它是**上层功能等价替换**，不是“相同低层模型”。出厂控制器内部的轮腿协同、
转向轨迹和稳定策略不可由当前手册完全推导，必须通过实机台架重新测量。MuJoCo 中得到的
侧偏、倒车补偿和最小滚动半径不能直接宣称适用于实机。

## 2. 采用的官方接口

当前同时实现 `basic_server` 与 direct ROS 2 `/NAV_CMD`，启动时只能二选一。默认选择
官方完整开发指南优先推荐的 `basic_server`；direct ROS 适合背部 Foxy 主机运行本项目的
自定义导航算法：

1. `src/drdds-背部主机当前版` 是已经部署过的 v1.2.0 ABI 真值。活动包 `src/drdds`
   已逐消息对齐：`MetaType` 使用 `builtin_interfaces/Time stamp`，`MotionInfoValue`
   的 state/gait 为扁平标量，`StdMsgInt32` 字段为 `value`；旧手册截图只作语义参考。
2. SDK 的低层关节、IMU、电池接口已合并进同一包，旧 vendored 同名包被可逆隔离；重新
   构建 SDK 和 MuJoCo 后端通过。
3. `basic_server` 的 APDU、JSON、端口、状态和心跳在手册中有完整定义，便于
   独立测试和版本审计。
4. direct ROS 后端使用 `/MOTION_INFO` 20 Hz 实测机体系速度，显式推进站立、RL 控制和
   步态，并复用同一 `cmd_vel_sdk`、ready/fault、enable 与安全契约。

### 2.1 已实现的协议参数

| 项目 | 官方约束 | 当前实现 |
| --- | --- | --- |
| 机器人地址 | `10.21.31.103` | launch 参数 `robot_host` |
| 速度通道 | UDP `30000`，Type 2/Cmd 25 | 20 Hz；只发送 X/Y/Yaw |
| 状态通道 | TCP `30001` | 接收 BasicStatus、MotionStatus 和异步设备错误 |
| APDU | 16 字节头，`eb 91 eb 90`，长度/帧号小端，JSON format 1 | 独立纯 Python codec 和分片 decoder |
| 心跳 | Type 100/Cmd 100，不低于 1 Hz，服务端约 2 s 失联 | 1 Hz |
| 固件速度看门狗 | 导航速度超过约 500 ms 未刷新则停车 | 本地命令 300 ms 超时，并持续发零速 |
| 运动状态 | 17 为 RL | 只有状态 17 才能 ready |
| 使用模式 | Type 1101/Cmd 5，导航 Mode 1 | enable 流程请求 Mode 1 |
| 步态 | 敏捷平地 `0x3002`/12290 | 默认 12290 |
| 启停 | Cmd 22；软急停状态 2 | 服务显式使能；急停发送零速和状态 2 |
| 步态切换 | Cmd 23，需停稳后切换 | MotionStatus 判停后发送 |
| 物理硬急停 | BasicStatus `HES`；红色尾部旋钮 | HES 缺失/触发均 fail-closed，释放后人工 disable→检查→enable |

官方 V1.1.7 及以上对敏捷平地步态给出的非零区间是：X 最小 0.15 m/s、Y 最小
0.25 m/s、Yaw 最小 0.35 rad/s，最大分别为 2.0 m/s、1.0 m/s、1.5 rad/s。当前第一版
把自主上限保守限制为 0.45 m/s、0.20 m/s、0.65 rad/s。低于官方非零区间的指令默认
归零，而不是在碰撞保护之后强行放大，以免执行未被安全层预测过的位移。

这个安全选择可能导致真机最终逼近时出现“低速指令被归零”。只有录制真实固件对小指令
的响应后，才能在安全层之前增加可预测的最小速度整形，或改用已验证的 passthrough。

## 3. 仿真与实机链路

### 3.1 MuJoCo 仿真

```text
SCAN cmd_vel_raw
  -> navigation_adapter
  -> collision_guard + safety_supervisor
  -> cmd_vel_safe
  -> locomotion_manager
  -> rl_deploy_cmdvel (本地 ONNX)
  -> /JOINTS_CMD
  -> MuJoCo 16 执行器
  -> /m20/sim/body_pose + joint/IMU feedback
```

### 3.2 M20 实机（两种最后一跳）

```text
SCAN cmd_vel_raw
  -> navigation_adapter
  -> collision_guard + safety_supervisor
  -> cmd_vel_safe
  -> locomotion_manager
  -> m20_basic_server_backend -> UDP Cmd25
     或 m20_direct_ros_backend -> /NAV_CMD
  -> AOS 出厂运动控制器
  -> 真实轮腿与机体
  -> TCP MotionStatus 或 /MOTION_INFO
  -> /m20/locomotion/measured_twist
```

真机 `MotionStatus` 或 `/MOTION_INFO` 只作为机体系速度反馈接入导航适配器，不被包装成
虚假全局里程计。
真实 `/m20/localization/body_pose` 必须由定位系统提供。

### 3.3 统一边界

| 契约 | 仿真 | 实机 |
| --- | --- | --- |
| 安全输入 | `/m20/control/cmd_vel_safe` | 相同 |
| 后端输入 | `/m20/locomotion/cmd_vel_sdk` | 相同 |
| 后端 ready | MuJoCo `/m20/sim/backend_ready` | `/m20/locomotion/backend_ready` |
| 后端 fault | MuJoCo `/m20/sim/backend_fault` | `/m20/locomotion/backend_fault` |
| 实测速度 | 仿真 Odometry twist | `MotionStatus` 或 `/MOTION_INFO` 转换的 TwistStamped |
| 全局位姿 | `/m20/sim/body_pose` | `/m20/localization/body_pose` |
| SCAN 点云 | PCD ray casting `/quad_0/cloud` | `/m20/sensing/cloud_map` |
| SCAN 传感器位姿 | `/quad_0/lidar_pose` | `/m20/localization/lidar_pose` |

同一启动图中只能存在一个执行后端和一个 `base_link` 位姿/TF 发布源。

## 4. 已完成的软件改造

### 4.1 官方运动后端

- 新增 `basic_server_protocol.py`：APDU codec、流式解码、状态解析和官方速度区间。
- 新增 `basic_server_backend_node.py`：TCP/UDP、心跳、状态机、看门狗、急停、ready/fault
  和 MotionStatus 反馈。
- 新增 `direct_ros_backend_node.py`：`/MOTION_STATE`、`/GAIT`、`/NAV_CMD` 显式状态机，
  `/MOTION_INFO` 实测速度、状态/步态确认及相同 fail-closed 健康接口。
- 新增 `/m20/hardware/enable_motion` `std_srvs/SetBool`。默认不开运动。
- `command_ownership_confirmed` 默认 false；未确认已停止机载规划与自动充电运动源时拒绝
  enable。
- 断网、状态超时、软急停、HES 硬急停、设备异步错误或状态/步态不一致都会让
  manager 保持零速。物理急停释放不会自动恢复运动。

### 4.2 能力配置隔离

- MuJoCo 继续使用 `m20_policy_v1_capabilities.yaml`，保留已有仿真实测参数。
- 真机使用 `m20_factory_agile_flat_capabilities.yaml`；只采用手册支持的接口能力和保守
  上限，不复制 MuJoCo 的漂移、倒车与转弯测量值。
- 官方 Cmd25 允许 X=Y=0 时给 Yaw，因此真机 profile 的几何模型允许原地偏航；这只是
  协议能力，实际转 90°/180° 的机体扫掠、轮腿滑移和场地需求仍待实机测定。
- 自动横移和倒车跟踪第一版保持关闭，即使协议支持 Y 与负 X，也不在未经测量时直接
  用于自主巡检。

### 4.3 数据源参数化

以下原来写死 `/m20/sim/body_pose` 的关键消费者已改为 launch 参数：

- SCAN local sensing、planner、closed-loop controller；
- grid route、navigation gateway；
- safety supervisor、collision guard；
- floor switch manager；
- navigation adapter 的速度反馈。

`inspection_mission_hardware.launch.py` 会关闭 PCD ray casting 和 RViz 运动学后端，改用
真实定位、实时点云、雷达位姿与官方运动后端。若这些真实话题没有发布，感知/定位安全门
不会 ready，机器人保持不动。

## 5. 除运动层外必须完成的真机改造

### 5.1 全局定位与 TF（阻塞项）

官方 MotionStatus 不是全局位姿。需要提供：

- 连续、带时间戳的 `nav_msgs/Odometry`；默认契约
  `/m20/localization/body_pose`；
- 一致的 `map -> odom -> base_link`，或明确由定位节点直接给出 map/world 位姿；
- `child_frame_id=base_link`，twist 为机体系速度；
- 定位健康、协方差、跳变和超时判据；
- 每层地图切换后的重定位/确认接口。

当前楼层管理器为兼容既有代码使用
`m20_warehouse_interfaces/srv/SetSimulationPose`。名称虽带 Simulation，但真机可以由一个
定位适配器实现同一 wire contract；更推荐实际电梯 profile 使用
`pose_handoff: wait_for_target` 或 `none`，由定位/运输 provider 明确报告到达，而不是
传送位姿。

### 5.2 激光雷达与 SCAN 输入（阻塞项）

手册中的合并雷达 `/LIDAR/POINTS` 为 `base_link` 坐标系、约 10 Hz。SCAN 需要的是与当前
地图/位姿时间一致的局部点云和传感器位姿，因此还需要：

- 明确使用机载合并点云，还是外部主机接收前/后 96 线雷达组播；
- 点云时间戳校验、去畸变、范围/高度/自体滤波；
- 正确的 lidar 外参与 TF；
- 转换到 SCAN 需要的 map/world 坐标并发布 `/m20/sensing/cloud_map`；
- 发布 `/m20/localization/lidar_pose`；
- 雷达失联、时间倒退或 TF 缺失时 fail-closed；
- rosbag 回放对比 PCD 仿真输入，确认 GridMap 更新频率和占据高度阈值。

旧 `m20_lidar_bridge` 仍被 `COLCON_IGNORE` 隔离，不能未经时间戳、TF 和 QoS 审计直接加入
生产启动图。

### 5.3 IMU 与时间同步（阻塞项）

- 官方 `/IMU` 约 200 Hz，已经转换到 `base_link`，应进入定位融合，而不是被当成全局
  姿态来源。
- 若外部主机直接收雷达组播，必须按手册配置 PTP/系统时间，核对 AOS、雷达、计算机与
  ROS 时间基准。
- 必须测量雷达、定位、速度状态和命令的端到端延迟；仿真中的 0 延迟假设不能沿用。

### 5.4 硬件安全与命令所有权（阻塞项）

- 确认机载路径规划与自动充电模块的停止流程。手册明确指出它们可能与 NAV_CMD/Cmd25
  争用运动控制。
- 把实体急停、遥控器接管、AOS 故障、电池/温度/电机故障接入系统 hold；软件急停不能
  替代机器狗硬件急停。
- 明确 basic_server 是否允许多个客户端；生产部署应只有一个最终速度发布者。
- 真实制动距离、通信中断停车时间和故障恢复必须在架空/支撑台、空旷场分级验证。

### 5.5 地图与多层运输

- 用实测 PCD/占据地图替换仿真随机障碍地图，校准墙面厚度与规划膨胀。
- 每层需要独立定位地图、lobby 坐标和 generation 交接规则。
- 实机电梯应实现 `ExecuteFloorTransfer` provider，并把 `transfer_adapter` 改成
  `external_action`；当前 `timed_hold` 只适合平面仿真。
- 确认电梯内雷达退化、金属反射、门运动、楼层识别、通信和重定位策略。

### 5.6 模型与可视化

- 官方 URDF 可以继续作为 RViz 外观与 TF 参考，但真实关节状态、传感器挂载外参和设备
  型号必须逐项核对。
- 真机不启动 MuJoCo/RViz 位姿积分器，不发布仿真 `/JOINTS_CMD`，也不让两个节点同时
  发布 odom/base TF。

## 6. 启动与切换设计

### 6.1 仿真入口保持不变

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash

ros2 launch m20_warehouse_inspection \
  inspection_mission_mujoco.launch.py
```

该入口仍使用 MuJoCo profile、仿真 PCD 感知和 `/m20/sim/body_pose`。

### 6.2 真机入口（目前仅用于接口联调）

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_hardware.launch.py \
  system_config:=/ABSOLUTE/PATH/TO/REAL_SITE.yaml
```

默认 `command_ownership_confirmed:=false`、`auto_enable_motion:=false`，因此即使网络连上也
不会进入可运动状态。完成所有前置检查后，才允许在明确参数
`command_ownership_confirmed:=true` 启动，并由现场人员调用：

```bash
ros2 service call /m20/hardware/enable_motion \
  std_srvs/srv/SetBool "{data: true}"
```

这不是当前可直接上机的操作许可。必须先完成第 7 节信息确认和分级台架测试。

## 7. 还需要用户/现场补充的信息

1. 机器狗准确型号、AOS/运动固件版本、`basic_server` 版本，是否不低于手册所述
   V1.1.7。
2. 部署程序运行在 GOS、NOS 还是外部工控机；网口、IP、ROS_DOMAIN_ID、RMW 和防火墙
   规划。
3. 若希望改用直接 ROS 2 运动接口，请提供实机：
   `ros2 interface show drdds/msg/NavCmd`、`MotionState`、`Gait`、`MotionInfo` 的输出和
   `/NAV_CMD`、`/MOTION_INFO` 的 QoS/频率实测。
4. 真实定位采用什么方案/功能包，当前能发布哪些 Odometry、Pose、TF、定位质量和
   重定位接口。
5. 使用 `/LIDAR/POINTS` 还是外部组播原始雷达；请提供一段静止、直行、转弯 rosbag，
   包含 PointCloud2、IMU、TF、MotionStatus/里程计。
6. 现场如何停止机载规划和自动充电运动源，以及遥控器/硬急停的接管规则。
7. 双层现场的地图、lobby 坐标、电梯接口和电梯内是否允许自主运动。

这些信息缺失时可以继续做协议模拟、rosbag 回放和离线算法验证，但不能声称已完成实机
运动、安全或多层部署验收。

## 8. 分级验收建议

1. **无机器人协议测试**：虚拟 basic_server 校验 APDU、心跳、状态机、超时与错误。
2. **架空/支撑台**：只验证连接、状态 17、步态、零速、急停和单轴短脉冲。
3. **空旷低速**：测 X/Y/Yaw 死区、符号、90°/180°扫掠、停车距离、MotionStatus 延迟。
4. **定位与雷达回放**：不使能运动，验证 TF、点云、GridMap、地图/代际切换。
5. **单障碍与宽通道**：低速闭环、实体急停随行，不直接从 0.9 m 窄道开始。
6. **窄通道和掉头**：用实测 footprint、漂移、制动和转弯数据更新真机 profile 后再测。
7. **单层巡检**：完整任务、失联、定位失效、雷达遮挡和遥控接管故障注入。
8. **多层运输**：电梯 provider、地图切换、重定位和故障保持全部通过后放行。

## 9. 本地手册依据

- `云深处山猫M20-pro-运动控制.pdf`
- `云深处山猫M20-pro-运动控制（basic-server).pdf`
- `云深处山猫M20 -pro-basic_server 通信协议总览.pdf`
- `云深处山猫M20-pro激光雷达.pdf`
- `山猫M20外部主机雷达数据接收配置指南 (1).pdf`
- `云深处山猫M20-pro-IMU.pdf`
- `云深处山猫M20-pro系统时间与时间同步.pdf`
- `云深处山猫M20-pro硬件参数.pdf`
- `云深处山猫M20-pro-辅助遥控.pdf`

手册支持“接口可用性”的判断；具体固件行为、实机几何能力和现场安全结论必须由对应
设备版本的实测数据补齐。

## 10. 本次软件回归结果

- `colcon build --symlink-install --packages-up-to m20_warehouse_inspection`：22/22 包成功。
- 运动、导航、巡检与集成包源代码测试：221/221 通过。
- 新增 basic_server 协议、真机 profile 与 hardware launch 针对性测试：27/27 通过。
- `m20_locomotion_control` 与 `m20_warehouse_inspection` flake8：61 个 Python 文件无问题；
  新运动包 copyright/PEP257 无问题。
- `inspection_mission_hardware.launch.py --show-args` 可正常展开，默认仍为
  `command_ownership_confirmed=false`、`auto_enable_motion=false`。
- 无界面 MuJoCo 使用原仿真入口完成 2.5 m 前进和回到起点两段导航，2/2 成功；终点
  误差 0.171 m/0.207 m，碰撞停止、障碍接触和后端故障均为 0。

详细数据见
`docs/test_reports/2026-08-10_official_motion_backend_simulation_regression.md`。这只证明本次
后端抽象没有破坏现有仿真链，不等价于 basic_server 已在真实机器狗上验收。
