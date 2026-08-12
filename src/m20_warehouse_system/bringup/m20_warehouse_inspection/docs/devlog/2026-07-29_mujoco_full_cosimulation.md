# 2026-07-29 M20 MuJoCo 完整联仿集成

## 1. 路线调整

本轮停止把 Gazebo 当作官方 RL 策略的必经执行环境，直接进入最终需要的第二级联仿：

```text
双区域 JSON/PCD
  -> 原版 SCAN 点云/规划/B 样条/闭环控制
  -> fail-closed safety
  -> m20_locomotion_control
  -> 官方 m20_sdk_deploy policy.onnx
  -> /JOINTS_CMD
  -> 官方 M20 MJCF + MuJoCo 接触动力学
  -> /JOINTS_DATA + /IMU_DATA + /m20/sim/body_pose
  -> SCAN 与任务闭环
```

RViz 继续是可视化工具，`pcl_render_node` 继续根据 active PCD 和真实物理位姿产生
实时局部点云。Gazebo 不参与这条主链。

## 2. 模块边界

### `m20_mujoco_backend`

新包完全隔离于任务和 SCAN：

- `models/m20_robot.xml` 是官方 M20 16 执行器 MJCF 的项目副本；
- STL 不重复复制，运行时从 `m20_official_description/meshes` 解析；
- `world_generator.py` 读取任意 system YAML 的 `floors.*.metadata_file`；
- 两个区域、所有障碍物和围栏一次加载进物理世界；
- `backend_node.py` 执行 SDK 的 PD/前馈命令并发布关节、IMU、odom、TF 和诊断；
- `/m20/sim/backend_ready` 与 `/m20/sim/backend_fault` 是上层放行契约。

### `m20_locomotion_control`

该包不修改 ONNX 动作，也不伪造离散轮/腿模式。它只完成：

- 安全速度到官方 SDK 速度话题的限幅；
- 直行、协调转向、横移意图标记；
- 高曲率时限制平移；
- 后端未就绪或物理故障时输出零速。

### `m20_warehouse_inspection`

新增 `motion_backend` 组合参数：

- `rviz`：保持历史平面运动学后端，供冻结回归使用；
- `external`：不启动平面后端，位姿、TF、关节状态由 MuJoCo 或实机提供。

`inspection_mission_mujoco.launch.py` 默认选择 0.70 m
`dense_four_corner_system.yaml`，并组合：

```text
inspection_mission_rviz.launch.py motion_backend:=external
m20_mujoco_backend/mujoco_backend.launch.py
m20_locomotion_control/sdk_locomotion.launch.py
```

## 3. 同源物理世界

生成器不从 PCD 反推盒子，也不另写一份障碍配置。它直接读取生成 PCD 时保留的 JSON：

- F1 246 个盒体；
- F2 246 个盒体；
- 每层边界根据 source bounds 生成；
- 相邻 `x=0` 边界重复段去重；
- `y=[-2,2]` 的共享门洞不生成碰撞墙；
- 地面是公共 MuJoCo plane。

0.70 m 默认高密度 profile 的结果：

```text
floor_count: 2
obstacle_count: 492
wall_count: 8
actuator_count: 16
MuJoCo ngeom: 545
```

active PCD 切换只影响感知和规划。物理世界始终包含两个区域，所以机器人必须实际运动
到共享原点并穿过门洞，不能通过切图跨越障碍或瞬移。

## 4. SDK/物理时序

官方 SDK 启动时会先发布 disable/reset/enable/status 等全零控制字。若后端把它们当作
电机命令，折叠态会失去支撑。因此后端使用以下状态：

1. 加载官方模型，设置官方折叠初始关节；
2. 腿关节使用启动 PD hold，轮子使用零速阻尼；
3. 忽略只有 control word、所有电机量为零的复位消息；
4. 收到第一个非零站立/电机命令后解除 startup hold；
5. 继续发布 `/JOINTS_DATA`、`/IMU_DATA` 给 SDK；
6. 基座高度大于 0.55 m 且姿态连续稳定 0.30 s 后才置 `backend_ready=true`；
7. ready 前不发布系统 body pose、TF 和 path，避免 SCAN 用折叠高度初始化；
8. SDK 命令超时后停止轮速目标并保留腿部稳定命令。

非有限状态、基座过低或 roll/pitch 超阈值会锁存 fault 并关闭 ready；速度适配器立即
进入 `FAULT_HOLD`。

## 5. 兼容性与移植

上层接口没有绑定 MuJoCo API。实机部署时可保持：

```text
/m20/control/cmd_vel_safe
  -> m20_locomotion_control
  -> 官方 SDK/硬件
```

并用实机定位替换 `/m20/sim/body_pose`。任务、楼层 generation、SCAN、PCD 切换和安全
状态机无需因后端变化而改写。MuJoCo 世界生成器和仿真诊断只在 simulation profile
启用。

## 6. 当前已知边界

- 官方零速度策略原先在本模型/摩擦参数下有约 `0.006 m/s` 慢漂。现已增加只作用于
  四个轮关节的停车阻尼制动；30 s 稳态窗口位移 `0.0000158 m`，但仍需 60 s/长时间
  多轮统计；
- 0.70 m 场景的 11 步任务已完成，但历史 `180 s` 单步超时使最后返航先进入
  `FAULT_HOLD`，经 `RETRY_CURRENT` 成功；MuJoCo profile 已独立调整为 `300 s`，
  新配置仍需一次无重试复验；
- 90°/180° 转向扫掠包络尚未形成统计报告；
- 当前 PCD 感知是 CPU ray casting，不是 MuJoCo 内部激光传感器；它仍由物理 odom
  实时驱动，足以验证当前完整导航/任务闭环；
- 完整节点图负载下实测实时率约 `0.883`，出现过可自动恢复的瞬时
  `ODOM_STALE`/`COMMAND_TIMEOUT`；
- 一次仿真结果不能直接作为实机转向半径或轮地摩擦结论。

## 7. 跟随视角与停车制动

- 原生 viewer 使用 `mjCAMERA_TRACKING` 跟随 `base_link`；
- 默认距离 4 m，完整入口通过 `mujoco_viewer_distance` 调整；
- `STOPPED/BACKEND_HOLD/FAULT_HOLD` 只覆盖四个轮子的目标速度、前馈和阻尼；
- 12 个腿关节继续使用官方 ONNX 输出维持平衡，浮动基座没有被冻结；
- `WHEEL_CRUISE/COORDINATED_TURN/LATERAL_MANEUVER` 到来时立即释放制动。

实测：

```text
30 s start: (-36.9985514, 0.0035275)
30 s end:   (-36.9985669, 0.0035243)
planar displacement: 0.0000158 m
average drift: 5.3e-7 m/s
```

随后发布约 3 s、`0.15 m/s` 的低速命令，日志确认制动释放，机器人实际前进约
`0.328 m`；命令结束后制动重新接合，后端仍为 ready、4 点接触且 fault 为空。

## 8. 完整任务结果

2026-07-29 无 GUI 执行 `dense_four_corner_patrol`：

- F1 四角全部完成；
- 机器人实际运动到共享原点，F1 generation 1 切换到 F2 generation 2；
- F2 四角全部完成；
- 机器人实际运动回共享原点，F2 generation 2 切回 F1 generation 3；
- 返航步骤在旧 `180 s` 超时后使用受支持的“重试当前步骤”，最终完成 11/11；
- 最终位姿 `(-37.0449, 0.0728, 0.5641)`，相对起点平面误差 `0.0855 m`；
- 最终后端无 fault、4 个接触、最大力矩约 `10.028 Nm`。

## 9. 下一步

1. 零速 60 s、直行/倒车、90°/180° 各 10 次台架；
2. 将实测扫掠包络反馈给动态 footprint/过道放行规则；
3. 使用新的 `300 s` MuJoCo 超时执行一次无重试 11 步复验；
4. 执行 `route_challenge` 绕障压力任务；
5. 长时间运行统计实时率、调度抖动和零速漂移；
6. 已固化 SDK 本地适配 patch 和新的 10 项目包/22 包独立工作空间闭包。
