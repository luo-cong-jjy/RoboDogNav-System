# 2026-07-29 M20 MuJoCo 完整联仿测试

## 范围

本报告覆盖：

- 同源 JSON 到 MJCF 世界生成；
- 官方 16 执行器模型加载；
- 启动关节保持；
- 官方 ONNX SDK 站立；
- 后端健康、接触和实时率；
- 原生 PCD local sensing；
- 低速速度链；
- 原版 SCAN 近距离目标闭环；
- 0.70 m 高密度双区域 11 步完整任务；
- F1→F2→F1 双向原点切图及最终返航；
- Python 单元、静态契约和风格检查。

不覆盖多轮 90°/180° 转向统计、路线受扰动压力任务或实机一致性。

## 静态与单元测试

```text
m20_mujoco_backend: 6 passed
m20_locomotion_control: 9 passed
m20_warehouse_inspection MuJoCo contracts: 4 passed
ament_flake8: 44 files across the three changed project packages, 0 errors
ament_pep257: 0 errors
workspace colcon test-result: 336 tests, 0 errors, 0 failures
```

世界生成校验：

```text
floors=2
obstacles=492
walls=8
actuators=16
nq=23
nv=22
nu=16
ngeom=545
```

两次相同输入生成的 XML 字节一致，F1/F2 首个碰撞体均可按 MuJoCo 名称检索。

## 构建

```text
colcon build --symlink-install --packages-select \
  m20_mujoco_backend m20_locomotion_control \
  m20_warehouse_inspection m20_sdk_deploy

4 packages finished
```

独立工作空间准备脚本随后从三个锁定 Git revision 和三个校验补丁重建源码树，只
加载 `/opt/ros/humble`，没有 source 当前工作空间，结果为：

```text
Project packages copied: 10
Resolved build closure: 22 packages
colcon build: 22 packages finished
```

六个 SCAN 上游包输出了未使用变量、符号位比较等既有编译 warning；没有编译或链接
失败。SDK 的 `rl_deploy` 与新增 `rl_deploy_cmdvel` 均从干净 checkout 编译成功。

## 后端独立稳定性

只启动 MuJoCo、尚未启动 SDK 时，机器人在官方折叠姿态和启动 PD hold 下稳定落地：

```text
base z: 0.109129 m
roll:  -8.3e-06 rad
pitch:  1.3e-05 rad
contacts: 28
max torque: 0.399 Nm
measured real-time factor: 0.99977
fault: empty
```

启动官方 SDK 后：

```text
startup_hold: false
base z: 0.564079 m
contacts: 4
max torque: 10.027 Nm
measured real-time factor: 0.99970
fault: empty
```

说明 SDK 已从折叠态进入站立/强化学习控制，四个轮端接地，物理后端没有通过直接设置
机身位姿来移动。

## 完整无 GUI 节点图

启动项：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_mujoco.launch.py \
  use_rviz:=false use_planner:=true use_mujoco_viewer:=false
```

观测到：

- 原版 `pcl_render_node` 收到 MuJoCo odom；
- F1 active PCD 为 174123 个 renderer 地图点；
- 首帧 local cloud 为 2044 点，持续帧约 2500 点；
- `/m20/sensing/state` 为 F1 generation 1、ready true；
- 后端、SDK、SCAN、controller、typed navigation、mission 和 floor switch 同图运行；
- 没有启动 `m20_rviz_kinematic_backend`，因此 body pose/TF 没有双发布者。

## 速度闭环

向 `/m20/navigation/cmd_vel_raw` 发布 `vx=0.20 m/s`、持续 5 s：

```text
before: (-36.3306, -0.2016, 0.5641)
after:  (-35.8854, -0.5201, 0.5646)
```

日志依次出现 safety `NAVIGATION`、locomotion `WHEEL_CRUISE`，命令结束后回到
`STOPPED`。位移由官方策略轮/腿动作和 MuJoCo 接触产生，不是 odom 积分器直接修改
位姿。

## 原版 SCAN 目标闭环

起点和目标：

```text
start: (-36.7769, -0.0167)
goal:  (-34.5000,  0.0000)
final: (-34.5085,  0.1065)
planar final error: 0.1069 m
```

SCAN 状态序列：

```text
INIT -> WAIT_TARGET -> GEN_NEW_TRAJ -> EXEC_TRAJ
-> REPLAN_TRAJ ... -> WAIT_TARGET
```

控制中观察到 `COORDINATED_TURN`、`WHEEL_CRUISE` 和
`LATERAL_MANEUVER`；最终：

```text
navigation state: IDLE
backend fault: empty
base z: 0.56408 m
contacts: 4
measured real-time factor: 0.99957
```

## 11 步完整任务

启动完整无 GUI 节点图后执行：

```bash
ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id dense_four_corner_patrol
```

实际状态序列覆盖：

```text
F1 左下 -> 右下 -> 右上 -> 左上
-> 物理行走至共享原点 -> F1 generation 1 切换到 F2 generation 2
-> F2 左下 -> 右下 -> 右上 -> 左上
-> 物理行走至共享原点 -> F2 generation 2 切回 F1 generation 3
-> 返回全流程起点 (-37, 0)
```

两次切图均在机器人到达共享原点并进入 hold 后发生；每次切换等待三帧新 generation
局部点云。物理世界始终同时保留两区，过程中没有写入基座位姿或传送机器人。

首轮使用历史统一单步超时 `180 s`。第 11 步返航在
`(-27.8722, -0.4992)` 触发 `FAULT_HOLD`，当时没有物理故障或碰撞锁存；通过任务
控制接口 `RETRY_CURRENT` 只重试当前步骤后成功完成。该步骤暴露的是官方策略物理
执行速度与平面后端超时预算不匹配，因此 MuJoCo 完整入口已把默认单步超时独立调整为
`300 s`，RViz 平面入口仍保持 `180 s`。

最终结果：

```text
mission state: COMPLETED
completed steps: 11/11
active floor: F1
map generation: 3
final pose: (-37.0449, 0.0728, 0.5641)
final planar error to (-37, 0): 0.0855 m
backend fault: empty
contacts: 4
max torque: 10.028 Nm
measured real-time factor under full load: 0.883
wall time including timeout and controlled retry: 1417.5 s
```

本轮证据可证明完整任务状态机、双向切图、物理行走和最终返航闭环通过；由于 300 秒
新默认值是在本轮后根据实测修正，不能把本次记录表述为“300 秒配置下一次通过”。

## 发现的问题

1. 初版 ready 在模型加载后立即为 true，使 SCAN 先看到折叠态 `z=0.20`。已修改为
   官方 SDK 站立高度/姿态连续稳定后再开放 body pose、TF 和导航速度。
2. `LocomotionManager` 曾误用 `_parameters` 覆盖 `rclpy.Node` 内部参数字典，节点启动
   失败。已改为 `_intent_parameters` 并补充关停保护。
3. Fast DDS 在当前受限测试环境中参与者发现不稳定；按项目既有 CycloneDDS 本机配置
   后话题发现正常。运行说明已固定两个终端必须使用相同 RMW/domain/URI。
4. 官方零速策略原先存在约 `0.006 m/s` 慢漂。已在 MuJoCo 执行层增加停车轮端
   阻尼制动，不冻结基座且保留腿部平衡；30 s 实测位移 `0.0000158 m`，平均
   `5.3e-7 m/s`。60 s 与多轮耐久统计仍保留为后续验收。
5. 临近目标的一次重规划出现多次 `1.5 m/s²` 动态可行性阈值重试，随后成功生成新轨迹
   并到达。这属于原 SCAN 参数与真实动力学响应的后续联合调参项。
6. 完整任务高负载时出现过瞬时 `ODOM_STALE`/`COMMAND_TIMEOUT`，随后自动恢复为
   `NAVIGATION`；全图同时运行时实测实时率约 0.883，后续耐久验收仍需统计渲染负载、
   ROS 调度抖动和 stale 阈值。
7. 最新 0.55 m 站立门控运行验证中，首帧系统里程计为 `z=0.563 m`，门控按预期
   在 SDK 稳定站立后开放。
8. 平面后端的 `180 s` 单步超时不适合官方策略物理执行；第 11 步通过受控重试完成。
   MuJoCo profile 默认值已独立改为 `300 s`，仍需在下一次无重试长任务中复验。

## 跟随相机与停车制动验证

原生 MuJoCo 窗口启动日志：

```text
MuJoCo viewer tracking base_link:
distance=4.00m, azimuth=135.0deg, elevation=-25.0deg
```

停车状态诊断为 `locomotion_mode=STOPPED`、
`parking_brake_active=true`。相隔 30 s 的平面位置变化为：

```text
(-36.9985514, 0.0035275)
-> (-36.9985669, 0.0035243)
distance: 0.0000158 m
```

短时 `0.15 m/s` 直行测试中状态按以下顺序变化：

```text
STOPPED / brake engaged
-> WHEEL_CRUISE / brake released
-> STOPPED / brake engaged
```

机器人实际位移约 `0.328 m`，最终后端 ready、fault 为空、4 点接触。

## 修订后冒烟验证

重新构建 `m20_inspection_core`、`m20_multifloor_map`、`m20_scan_navigation` 和
`m20_warehouse_inspection` 后：

```text
4 packages finished
336 tests, 0 errors, 0 failures, 0 skipped
installed launch argument navigation_timeout_sec: 300.0
```

短时启动完整节点图并在后端 ready 后发送 Ctrl-C，地图、感知、安全、任务、切层、
MuJoCo、运控和官方 SDK 项目节点均 clean exit。ROS 2 自带
`static_transform_publisher` 仍由 SIGINT 以 `-2` 退出并被 launch 标为 error，这是
外部进程的信号退出表现，不是项目节点异常或物理故障。

## 结论

第二级完整联仿主流程通过：同源场景、原版 SCAN、官方 ONNX 运控、官方 M20 MuJoCo
动力学和系统 odom 已连通；0.70 m 高密度双区域任务完成 11/11，两次共享原点切图
均由物理行走触发，最终返回起点误差 0.0855 m。

当前仍不宣称已通过 300 秒配置的一次性无重试长任务、多轮转向稳定性、路线挑战或
实机等价验收；这些边界与本次主流程通过结论分开管理。
