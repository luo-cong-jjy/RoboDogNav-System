# 2026-07-31 Go2 与 M20 运动 SDK、模型及 SCAN 执行边界对照

## 结论先行

两台机器人的上层运动接口都可以抽象成机体系
`forward / side / yaw`，但接口以下的职责完全不同：

- Go2 官方 `SportClient::Move(vx, vy, vyaw)` 调用机载 Sport 服务，步态生成、
  足端落点和 12 关节控制由机器人内部运控完成。
- 当前可核实的 M20 开源 `sdk_deploy` 不是同层级的机载导航服务。它在外部计算机
  运行 `policy.onnx`，把三维速度意图变成 12 个腿关节位置目标和 4 个轮关节速度
  目标，再经 `/JOINTS_CMD` 下发。
- 原版 SCAN-Planner 仓库没有包含 Unitree SDK 适配器。实机模式只发布
  `geometry_msgs/Twist /cmd_vel`，要求外部驱动接管；默认仿真则直接对
  `vx / vy / wz` 做 100 Hz 平面数值积分。

因此，原版 RViz 中 Go2 的“横移、原地转向、严格贴合 B-spline”不是 Go2
真实动力学或官方 Sport SDK 的仿真结果，而是理想全向执行器的结果。当前 M20
出现转弯漂移、轨迹落后和碰撞保护恢复，不能通过继续修改 SCAN 膨胀参数来掩盖，
需要在独立的 M20 平台执行层解决。

## 对照范围与来源

本轮区分三个容易混淆的对象：

1. SCAN-Planner 原仓库 `origin/main` 的 Go2 仿真与实机接口；
2. Unitree 官方公开的 Go2 SDK2/Sport ROS 2 接口；
3. Deep Robotics 官方公开 `sdk_deploy` 中的 M20 策略部署接口，以及本项目增加的
   `/cmd_vel` 桥。

本地固定版本：

```text
SCAN-Planner origin/main: e3f28a2
SCAN-Planner ros2-community: d0b921c
DeepRoboticsLab/sdk_deploy: ee289d4
```

官方公开资料：

- [SCAN-Planner 原仓库](https://github.com/wuyi2121/SCAN-Planner)
- [Unitree Go2 SportClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp)
- [Unitree ROS 2 接口](https://github.com/unitreerobotics/unitree_ros2)
- [Deep Robotics sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy)
- [Unitree Go2 产品参数](https://www.unitree.com/go2/)
- [Deep Robotics M20 产品参数](https://www.deeprobotics.cn/robot/index/lynx.html)

“M20 没有某项能力”的表述仅限当前公开 `sdk_deploy`。商业版机载固件是否另有
高层运动服务，需要以实际交付版本的 M20 接口手册和真机话题为准。

## 三条实际执行链

### SCAN 默认仿真

```text
B-spline
  -> closed_loop_controller（100 Hz）
  -> /quad_0/cmd_vel: vx, vy, wz
  -> go2_kinematic_sim（100 Hz）
  -> x += vx_world * dt
     y += vy_world * dt
     yaw += wz * dt
  -> /quad_0/body_pose

/quad_0/body_pose
  -> go2_gait_publisher
  -> /joint_states（仅 RViz 腿部动画）
```

`go2_gait_publisher` 只根据已经产生的平面位姿速度绘制摆腿动画。关节动画不参与
机身运动、接触、力矩或碰撞计算。单独提供的 Go2 Gazebo 包也只有 12 关节位置轨迹
控制器，没有把 `/cmd_vel` 转换为真实步态的控制器，而且不在 SCAN 默认入口中启动。

### Go2 官方实机接口

```text
上层导航 Twist
  -> 外部 Twist/Sport 适配器（SCAN 仓库未提供）
  -> SportClient::Move(vx, vy, vyaw)
  -> /api/sport/request
  -> Go2 机载 Sport/步态控制
  -> 12 个腿关节
  -> /sportmodestate
```

官方 Sport 接口还提供 `StopMove`、`BalanceStand`、`RecoveryStand`、
`StaticWalk`、`TrotRun`、`EconomicGait` 等高层动作。`SportModeState`
可反馈机身速度、偏航速度、步态类型、足端位置/速度和足端力。

Go2 也提供 `/lowcmd`，可直接向电机发送
`q / dq / tau / kp / kd`，但这与 `SportClient::Move` 是不同控制层。对导航项目，
使用 Sport 层意味着低层步态可实现性由官方机载控制器负责，而不是由 SCAN 负责。

### 当前 M20 完整联仿与后续实机链

```text
SCAN B-spline
  -> closed_loop_controller
  -> /m20/navigation/cmd_vel_raw
  -> M20 navigation adapter
  -> collision guard + safety supervisor
  -> /m20/control/cmd_vel_safe
  -> locomotion manager
  -> /m20/locomotion/cmd_vel_sdk
  -> 项目增加的 CmdVelInterface
  -> UserCommand(forward, side, yaw)
  -> M20PolicyRunner / policy.onnx（约 50 Hz）
  -> 12 个腿关节位置目标 + 4 个轮速目标
  -> /JOINTS_CMD
  -> MuJoCo 或 M20 底层 DDS
```

M20 官方仓库原入口使用键盘或手柄填充 `UserCommand`，没有 ROS 标准
`geometry_msgs/Twist` 订阅器。本项目的 `CmdVelInterface` 与自动起立状态机位于
`M20_sdk_deploy/extensions`，属于导航集成层，不应误记为官方原生 Sport 服务。

## SDK 接口逐项对照

| 项目 | Go2 官方 Sport 层 | M20 公开 `sdk_deploy` |
|---|---|---|
| 高层速度输入 | `Move(vx, vy, vyaw)` | `UserCommand.forward/side/turnning_vel_scale` |
| ROS 2 高层入口 | `/api/sport/request` | 官方原包无 `Twist` 入口；本项目增加 `/m20/locomotion/cmd_vel_sdk` |
| 步态/轮腿协调位置 | Go2 机载固件内部 | 外部 `policy.onnx` |
| 用户可选运动模式 | 多个 Sport 动作和步态接口 | `Idle/StandUp/RLControl/LieDown/Damping`，没有轮/腿二选一命令 |
| 高层停止 | `StopMove`、`BalanceStand` | 三维速度置零后策略仍持续平衡，不能把 16 电机命令整体清零 |
| 高层状态反馈 | 速度、偏航、步态、足端状态等 | 公开包主要提供 IMU、16 关节和电池；机身里程计需外部定位 |
| 低层命令 | `/lowcmd`，每电机 `q/dq/tau/kp/kd` | `/JOINTS_CMD`，每关节 `position/velocity/torque/kp/kd` |
| 导航栈与底层隔离 | 高：机载 Sport 服务吸收大部分步态差异 | 较低：项目直接承担策略进程、状态机、超时和关节通道 |

两者低层接口形式相近，但当前系统使用的层级不对称：Go2 导航通常对接官方高层
Sport 服务；M20 当前则把公开 RL 部署策略和低层关节通道一起纳入了本项目。

## 模型与运动约束对照

| 项目 | 本地 Go2 模型 | 官方 M20 模型 |
|---|---:|---:|
| 结构 | 纯四足 | 轮足 |
| 执行器 | 12 个腿关节，每腿 3 个 | 16 个，每腿 3 个腿关节 + 1 个轮关节 |
| 末端接触 | 半径 0.02 m 足端 | 半径 0.09 m、宽 0.054 m 车轮 |
| 模型总质量近似 | 16.17 kg | URDF 约 34.49 kg；产品标称 35 kg |
| 产品站立尺寸 | 官方约 0.70 × 0.31 × 0.40 m | 官方约 0.82 × 0.43 × 0.57 m |
| 本项目安全外廓 | 原 SCAN 使用 0.25 m 双圆 | 使用用户确认的 0.82 × 0.51 × 0.57 m 外廓，0.30 m 连续保护双圆 |
| 腿长 | 大腿/小腿各约 0.213 m | 大腿/小腿各约 0.25 m |
| 转向机制 | 调整四足落点和支撑力 | 固定轴车轮差速/滑移作用与腿部姿态、接触协同 |
| 直接横移 | 足端可横向重置，仍受步态约束 | 车轮无转向关节，横移必须依靠腿部姿态和接触滑移，不能按全向轮理解 |

M20 四个轮轴均固定在机体横向，模型中没有转向舵机。因此：

- `vx + wz` 是合理的滚动曲线输入，轮速和腿部动作可以同时参与；
- `vy` 虽然是策略训练输入，但不等于可以像麦克纳姆轮或理想质点那样瞬时横移；
- `vx=0, wz!=0` 不是数学意义上的零空间旋转，必须通过轮地滑移和腿部协调产生，
  会有有限扫掠、漂移和姿态变化；
- 质量约为 Go2 的两倍，M20 机身惯量和持续轮地接触使相同 `wz` 指令不能直接沿用
  理想 Go2 仿真的瞬时响应假设。

## M20 策略内部到底怎样用腿和轮

官方策略每约 20 ms 构造一次 57 维观测：

```text
3  base angular velocity
3  projected gravity
3  command(forward, side, yaw)
16 joint position error
16 joint velocity
16 previous action
= 57
```

ONNX 同时输出 16 维 action：

- 每条腿前三维经 `0.125 / 0.25 / 0.25` 缩放，成为相对默认站姿的关节位置目标；
- 每条腿第四维经 `5.0` 缩放，成为轮关节速度目标；
- 腿部使用 `kp=80, kd=2`；
- 轮部使用 `kp=0, kd=0.6`。

所以“直线只用轮、转弯才用腿”不是当前策略的接口契约。腿始终承担站立和平衡，
轮和腿也允许在同一帧共同产生转弯。项目中的 `WHEEL_CRUISE`、
`COORDINATED_TURN` 和 `LATERAL_MANEUVER` 只是命令整形与诊断标签，未进入
ONNX 观测，不能强制切换策略内部步态。

## 为什么原 SCAN 看起来更灵活

原控制器本身包含两个对理想执行器非常友好、对 M20 较苛刻的假设：

1. 世界系轨迹误差转到机体系后，同时输出独立的 `vx` 和 `vy`；
2. 航向误差超过 `0.8 rad` 时冻结轨迹时间，并输出 `vx=vy=0` 的纯偏航。

理想仿真立即、无误差地兑现这三个分量，且没有加速度、惯量、滑移、足迹扫掠或碰撞。
真实 Go2 若通过 Sport 接口执行，机载步态控制仍会在脚下完成一层可实现性映射；
M20 当前公开部署链则把该映射暴露为我们正在测试的 ONNX 与物理接触问题。

此前冷启动矩阵已经验证：

- M20 当前模型/策略的低速纯偏航存在明显下沉、漂移甚至反向响应；
- `(vx,wz)=(0.35,0.65)` 连续五次保持在稳定内边界；
- 对应最小中心线半径约 `0.54 m`，计入 0.82 × 0.51 m 外廓后的稳态外侧扫掠
  半径约 `0.89 m`；
- 0.90 m 障碍间距足以验证直行通道，但不代表能够在该通道内完成 90°/180° 转身。

这正是原版质点可以贴着规划线旋转、M20 却可能先偏离再触发保护的根本差异。

## 对当前系统的直接设计结论

### 保持不变

- SCAN 点云、GridMap、A*、B-spline、重规划和原参数继续冻结；
- 地图 0.90 m 最小实体间距继续作为直行压力场景默认值；
- 0.30 m 连续双圆保护、实体接触监测和安全 supervisor 保持独立；
- `cmd_vel`、安全门、任务 Action 和地图切换契约保持平台无关。

### 下一阶段应新增的平台能力契约

在 `m20_locomotion_control` 中把零散参数收敛为明确的
`LocomotionCapabilityProfile`：

```yaml
platform: m20_policy_v1
supports_command_vy: bounded
supports_zero_radius_yaw: false
stable_turn_min_forward: 0.35
stable_turn_max_forward: 0.45
max_yaw_rate: 0.65
minimum_centerline_turn_radius: 0.54
turn_swept_radius: 0.89
body_velocity_feedback_source: external_odometry
```

同一接口以后可以增加：

```yaml
platform: go2_sport
backend: sport_client
stop_primitive: StopMove
state_feedback: sportmodestate
```

能力 profile 的值必须来自重复实测，不因为 SDK 函数签名相同就共享。

### M20 命令投影原则

1. SCAN 的纯偏航请求投影到已经验证的滚动弧线，不能原样下发；
2. `vy` 只作为有界低速修正或人工调试输入，不能假设与 `vx` 同等可实现；
3. 转弯是否允许必须检查完整扫掠半径，而不只检查当前静态双圆；
4. 恢复动作需要时间、距离和进展上限，不能无限重复“预测停车—直退—再规划”；
5. 实测速度反馈必须来自 MuJoCo `base_link` twist 或实机 LIO/融合里程计。公开 M20
   SDK 的 IMU/关节反馈本身不能替代平面机身速度；
6. 零速度仍保留 RL 腿部平衡，只制动轮速；这与 Go2 `StopMove` 的调用语义分开实现。

## 下一步实现顺序

1. 提取 `LocomotionCapabilityProfile`，让适配器、安全预测和测试探针使用同一份
   M20 运动包线；
2. 为纯偏航建立显式 `ROLLING_TURN` 投影，并给恢复增加最大持续时间、最大位移和
   必须取得的净空/目标进展；
3. 用现有单障碍任务做无反馈/有反馈 A/B，验收零实体接触、最小保护净空和恢复次数；
4. 通过后再运行六目标与双场景完整巡检；
5. 真机接入前现场读取 M20 实际高层接口、里程计和安全服务。如果交付固件提供类似
   Go2 Sport 的官方高层服务，则新增 backend，不改 SCAN 和任务层。

## 实施状态补记

上述第 1、2 项已经完成：`m20_policy_v1` 成为单一平台能力配置，碰撞恢复具有
`6.0 s / 0.75 m / 1.50 s` 有界退出，并且恢复预算只有在连续 clear `1.50 s`
后才重新许可。进一步动态复现表明，180° 回程不应投影成向前滚动掉头；完整 M20
入口现按 B-spline 切向选择前进或倒车执行，独立 SCAN 默认行为保持不变。

官方 SDK + MuJoCo 六目标复验已达到 `6/6`，两个回程段均以 REVERSE 完成，且无
恢复、预算耗尽或实体障碍接触。详细数据见
`docs/test_reports/2026-07-31_m20_bidirectional_tracking_bounded_recovery.md`。下一项仍是
在新开环基线上重做速度闭环 A/B，而不是把旧闭环结果直接提升为默认值。

这条路线保留了原 SCAN 的算法价值，同时把 Go2 和 M20 真正不同的运动学、运控层级
及反馈能力限制在可替换的平台适配层内。
