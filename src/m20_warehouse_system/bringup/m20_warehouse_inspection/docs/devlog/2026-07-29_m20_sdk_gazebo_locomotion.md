# 2026-07-29 M20 SDK 运控与 Gazebo 迁移审计

> 路线更新：本文件保留 6A-1 当时的 Gazebo 可行性审计。项目随后选择直接使用
> MuJoCo 完成最终联仿，Gazebo 已降为可选兼容项。当前实现与验收见
> `2026-07-29_mujoco_full_cosimulation.md`。

## 1. 本次目标

解决 RViz 平面运动学后端把 M20 当作完整全向刚体积分、原地偏航时腿部不参与的问题。
目标不是在显示层伪造四足动作，而是把现有 SCAN 规划结果通过安全层接入官方 M20
RL 运控，使策略真实输出 12 个腿关节和 4 个轮关节命令，并判断同一部署控制器迁移到
Gazebo 是否可行。

本次不改冻结的 RViz 1.3.0 场景、地图、SCAN 参数、任务或楼层切换链路。

## 2. 官方 SDK 审计结果

审计对象：

```text
src/third_party/sdk_deploy
revision: ee289d475f2dedf0332b7542f2b173fa9e8d1456
license: BSD-3-Clause
policy sha256:
e63169b7727d197abc952c626d02458f31859a627f811660c14a0048fa4f5302
```

当前 vendor 工作树已有大量本地修改，因此不能把它误称为干净的官方快照。项目代码
不再继续直接修改该目录；正式冻结 SDK profile 前必须把官方基准、项目 patch 和生成
文件分开记录。

官方运控数据链：

```text
UserCommand(forward, side, yaw)
  -> M20PolicyRunner(policy.onnx)
  -> 16 joint actions
  -> leg position PD + wheel velocity/damping command
  -> M20Interface
  -> /JOINTS_CMD
```

策略观测为 57 维：

- 机身角速度 3 维；
- 重力在机身坐标系的投影 3 维；
- 前进、横移、偏航命令 3 维；
- 16 维关节位置误差；
- 16 维关节速度；
- 16 维上一时刻动作。

ONNX 输入为 `obs[1,57]`，输出为 `actions[1,16]`。16 个输出最终映射为每条腿
`hipx/hipy/knee/wheel`。腿关节使用位置 PD，轮关节强制 `kp=0`，使用速度/阻尼控制。
策略推理周期 20 ms，状态机主周期 5 ms。

由此得到两个边界：

1. 策略不依赖 MuJoCo API，只依赖关节状态、IMU 和关节执行接口，因此可以移植到
   Gazebo。
2. 策略输入没有 `WHEEL/LEG` 离散模式。上层可以识别直行巡航、协调转弯和横移意图，
   但十二个腿关节与四个轮关节如何配合仍由当前 RL 策略决定。若未来必须强制指定
   “仅轮式”或某种四足步态，需要官方多策略接口或重新训练带 mode 条件的策略。

## 3. 当前本地 SDK 扩展

vendor 包内已有一个非官方的 `extensions/`：

```text
/cmd_vel
  -> CmdVelInterface
  -> UserCommand
  -> 官方状态机和 policy.onnx
  -> /JOINTS_CMD
```

这说明 SCAN 的 `geometry_msgs/msg/Twist` 接入方向已经成立，但当前实现有三个问题：

- 扩展直接位于 `third_party`，不满足 vendor 只读和项目隔离要求；
- 默认话题 `/cmd_vel` 没有表达“必须先经过安全层”的契约；
- 它不能替代 Gazebo 关节动力学后端。

本次没有复制或修改官方 ONNX、零位、关节方向、状态机和硬件接口。新增
`m20_locomotion_control` 作为项目自有边界，vendor SDK 继续作为可替换、可锁定的
第三方依赖。

## 4. Gazebo 可行性结论

结论：技术可行，但不能把当前 Gazebo 模型直接接到 SDK。

MuJoCo 桥当前完成的工作是：

```text
/JOINTS_CMD
  -> kp * (q_des - q) + kd * (dq_des - dq) + tau_ff
  -> 16 actuator torque

MuJoCo joint/IMU sensors
  -> /JOINTS_DATA + /IMU_DATA
  -> M20Interface
```

Gazebo 只要实现同一语义接口，RL 部署节点就不需要知道仿真器已经改变。当前环境已有
Gazebo Classic、`gazebo_ros` 和 `gazebo_ros2_control`，但缺少项目所需的 M20
`/JOINTS_CMD`、`/JOINTS_DATA`、`/IMU_DATA` 兼容桥。

现有两个 Gazebo 运动插件均不适合 RL 验证：

- `m20_legged_kinematic_plugin` 关闭重力/碰撞并直接设置世界位姿，只做腿部动画；
- `m20_four_wheel_drive_plugin` 固定腿部并按差速轮模型驱动，不执行 16 维 RL 动作。

Gazebo RL 模型必须停用上述插件，保留官方 16 关节树，启用重力、碰撞、惯性和接触，
再增加高频关节 PD/力矩桥。

## 5. MuJoCo 到 Gazebo 的不可忽略差异

直接 sim-to-sim 迁移不保证第一次就稳定，至少需要匹配：

- 1 kHz 物理/关节执行周期和 200 Hz 状态反馈；
- 16 关节顺序、方向和机械零位；
- 机身 IMU 坐标系、四元数/RPY 单位与角速度符号；
- 腿部 `76.4 Nm`、轮部 `21.6 Nm` 力矩上限；
- 轮地摩擦、侧向滑动、接触刚度和阻尼；
- 关节阻尼、惯量、执行延迟、碰撞几何和自碰撞策略；
- Gazebo ODE 与 MuJoCo 接触求解差异。

因此“能加载模型”不能作为迁移成功。必须按静止、起立、低速直行、低速偏航、
90°/180° 转向、规划闭环的顺序逐级放行。

## 6. 新增项目适配边界

新增包：

```text
m20_locomotion_control
```

数据链：

```text
SCAN
  -> /m20/navigation/cmd_vel_raw
  -> safety supervisor
  -> /m20/control/cmd_vel_safe
  -> m20_locomotion_manager
  -> /m20/locomotion/cmd_vel_sdk
  -> m20_sdk_deploy/rl_deploy_cmdvel
  -> /JOINTS_CMD
```

`m20_locomotion_manager` 的约束：

- 只订阅安全速度，不接 SCAN 原始速度；
- 不放大任何速度分量；
- 对 NaN/Inf 置零；
- 300 ms 命令超时置零；
- 输出 `STOPPED`、`WHEEL_CRUISE`、`COORDINATED_TURN`、
  `LATERAL_MANEUVER` 意图；
- 高曲率/近原地转向时限制前进速度，减少狭窄区扫掠；
- 明确说明 mode 是上层意图，不是假装 ONNX 已有离散步态输入。

SDK 启动默认关闭。只有 MuJoCo、后续 Gazebo 动力学桥或实机后端已经发布可靠
`/JOINTS_DATA` 与 `/IMU_DATA` 时，才允许 `start_sdk:=true`。

## 7. 下一步实施顺序

### 6A-1：已完成

- 完成 SDK/ONNX/消息接口审计；
- 确认 Gazebo 迁移可行性和限制；
- 建立隔离的安全速度到 SDK 速度适配包；
- 冻结首版意图阈值与单元测试。

### 6A-2：Gazebo RL 动力学桥

- 从官方 M20 URDF 建立独立 xacro/SDF wrapper，不改官方原件；
- 删除运动学和四轮差速插件；
- 实现 16 关节命令订阅、力矩限幅和 world-update PD；
- 发布与 SDK 坐标语义一致的关节状态、IMU、电池占位状态；
- 保证同一时间 `/JOINTS_CMD` 只有一个发布者、只有一个执行后端。

### 6A-3：单体姿态与转向台架

- 60 s 静止/起立不倾倒；
- `vx=+0.1/-0.1 m/s` 直行/倒车；
- `yaw=+0.15/-0.15 rad/s` 低速偏航；
- 分别测量 90°、180° 转向时间、质心漂移、扫掠包络、最大 roll/pitch；
- 至少 10 次重复，记录关节限位、力矩饱和、接触失稳和跌倒。

建议第一轮放行阈值：

- 60 s 内不跌倒；
- 静止平移漂移小于 0.05 m；
- roll/pitch 峰值小于 10°；
- 90°/180° 转向不越过关节限位；
- 转向实测扫掠包络进入 SCAN/碰撞保护的动态 footprint 参数。

### 6A-4：接回双区域系统

- 用 Gazebo odom 替换 `/m20/sim/body_pose`；
- PCD local sensing 和双区域切图保持不变；
- 先跑手动 RViz 目标，再跑 11 步巡检；
- 对急停、任务 hold、切图 hold 和超时停车做同一套 typed 回归。

完成 6A-4 后再决定是否接 Gazebo 雷达。Gazebo 动力学尚未通过前，不把 RL 后端作为
日常启动默认项，也不把其转向半径当作真机最终参数。
