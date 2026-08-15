# M20速度参数对齐试验与默认回退

日期：2026-08-14

## 最终状态（恢复已验证基线）

本文件记录一次已完成的 A/B 试验。试验将 SCAN 的轨迹参数化和闭环输出上限
同步降低至 M20 能力包络，并在相同目标复验中取得当前更好的实体执行结果。
2026-08-14 复核后恢复该组参数为完整 M20 系统默认值。

自本次回退起：

- 完整 RViz/MuJoCo 启动先加载 `scan_vendor_*`，随后仅叠加
  `scan_m20_velocity_*.yaml` 中的五个速度键；
- 独立 `m20_scan_navigation/f1_scan.launch.py` 仍默认使用原生 SCAN 参数，
  用于基线对照；
- M20 的 `0.45/0.20/0.65` 速度包络只由下游运动适配与安全限幅层执行；
- 不恢复 PGM/YAML、二维栅格路线或已删除的旧兜底规划链。

## 试验修改边界

本轮只统一完整 M20 系统中的速度参数，不修改第三方 SCAN Planner 源码、空间
路径算法、重规划状态机、碰撞膨胀、优化权重、控制增益、航向逻辑或轨迹时钟。

独立启动 `m20_scan_navigation/f1_scan.launch.py` 始终默认加载原生
`scan_vendor_*` 参数。试验期间完整仓库 RViz/MuJoCo 入口曾显式叠加两个只包含
速度键的 M20 配置文件；该默认覆盖现已撤销。

## A/B试验中的速度链

| 层级 | 参数 | 默认值 |
|---|---|---:|
| SCAN轨迹参数化 | `manager.max_vel` | 0.45 m/s |
| SCAN优化可行性 | `optimization.max_vel` | 0.45 m/s |
| 闭环纵向输出 | `max_vx` | 0.45 m/s |
| 闭环横向输出 | `max_vy` | 0.20 m/s |
| 闭环偏航输出 | `max_vyaw` | 0.65 rad/s |
| 安全监督器 | `max_linear_x/y`, `max_angular_z` | 0.45/0.20/0.65 |
| M20运动管理器 | `max_forward/side/yaw` | 0.45/0.20/0.65 |
| 官方SDK策略入口 | `max_forward/side/yaw` | 0.45/0.20/0.65 |

数值由 `m20_policy_v1_capabilities.yaml` 的已验证命令包络确定。SCAN Planner 和
closed-loop controller 使用独立的速度覆盖文件，是因为它们分别属于两个 ROS
节点；其他下游节点继续从同一个能力配置读取。

## 显式切回原生速度基线

如需显式复现上游 0.75 m/s 速度基线，可覆盖以下两个参数路径：

```bash
ros2 launch m20_warehouse_inspection inspection_mission_mujoco.launch.py \
  planner_config:=$(ros2 pkg prefix m20_scan_navigation)/share/m20_scan_navigation/config/scan_vendor_planner.yaml \
  controller_config:=$(ros2 pkg prefix m20_scan_navigation)/share/m20_scan_navigation/config/scan_vendor_controller.yaml
```

不传这两个参数时，主启动命令使用已验证的 M20 速度覆盖。

## 验证结果

### 构建与静态回归

- `m20_scan_navigation`、`m20_locomotion_control`、
  `m20_warehouse_inspection` 均已重新构建；
- 相关源码契约定向测试：58 项通过；
- 三个包的 `colcon test`：34 个 CTest 目标全部通过。

### 完整MuJoCo同目标复验

测试目标：`(-26.5977, -4.89832)`；该目标与修改前失败基线相同。

| 指标 | 修改前原生速度链 | 本轮速度统一后 |
|---|---:|---:|
| 60秒内到达 | 否 | 是 |
| 到达时间 | 超过60秒 | 38.10秒 |
| 最终位置误差 | 7.221 m | 0.119 m |
| 障碍接触物理步事件 | 3433 | 51 |
| 障碍接触峰值力 | 774.6 N | 149.3 N |

结论：速度统一显著消除了规划轨迹时间与 M20 实际能力之间的差距，并使该任务
成功完成；但仍存在约 0.2 秒的短暂实体擦碰，尚未达到“零接触”验收标准。本轮按
修改边界不继续调整规划或控制逻辑，后续应单独定位动态扫掠包络问题。

原始复验数据保存在：

- `/tmp/m20_velocity_aligned_turn/navigation_motion_summary.json`
- `/tmp/m20_velocity_aligned_turn/navigation_motion_samples.csv`
- `/tmp/m20_velocity_aligned_turn_launch.log`

## 原生速度回退对照（已否决为默认）

回退默认值后，在 `ROS_DOMAIN_ID=181`、`ROS_LOCALHOST_ONLY=1` 的隔离域中启动
完整无界面 MuJoCo 链路，使用相同起点和目标 `(-26.5977, -4.89832)` 复验。

| 指标 | 速度统一试验 | 回退原生SCAN默认后 |
|---|---:|---:|
| 60秒内到达 | 是，38.10秒 | 否 |
| 最终位置误差 | 0.119 m | 7.202 m |
| 航向误差均值/P95 | 16.21°/43.53° | 2.52°/3.42° |
| 障碍接触物理步事件 | 51 | 3446 |
| 障碍接触峰值力 | 149.3 N | 709.7 N |
| B样条轨迹消息 | 20 | 36 |

回退后的首次实体接触发生在约 `13.755 s`、位置
`(-33.186, -1.674)`。此时 SCAN 原始纵向命令约为 `0.75 m/s`，M20 运动层按
能力包络裁剪为 `0.45 m/s`。接触持续到测试结束；日志中没有对应的 SCAN A-star
失败，说明该次停止首先是“轨迹按原生时间继续推进、实体执行速度被下游裁剪”形成
的执行时基失配和物理顶障，而不是本轮改坏了 A-star。

因此当前默认值完成了“恢复原生 SCAN 规划和控制语义”的目标，但没有通过 M20
实体零接触验收。后续若继续优化，应保持空间规划主链不动，在运动适配边界解决轨迹
时间推进与实测位姿进度不一致的问题；不能仅靠下游速度截断，也不能把本轮回退结果
表述为已经解决碰撞。

回退复验数据保存在：

- `/tmp/m20_scan_vendor_default_restore/navigation_motion_summary.json`
- `/tmp/m20_scan_vendor_default_restore/navigation_motion_samples.csv`
- `/tmp/m20_scan_vendor_default_restore_launch.log`
