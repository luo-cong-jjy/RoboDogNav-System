# 2026-07-30 动态净空与单障碍保护参数整定

## 目标

本轮解决两个相互关联的问题：

1. 用官方 M20、SDK 策略和 MuJoCo 动力学实测 0.75/0.80/0.90 m
   窄通道，不再把静态栅格连通当作动态可通过；
2. 验证原生 SCAN 绕行单个障碍时，独立保护器不会让双圆机身进入膨胀保护区，
   同时避免过长的恒指令预测造成安全停车死锁。

本轮没有缩小 M20 足迹，也没有恢复已经撤回的 `0.20 m` 栅格路线膨胀。
可选栅格路线在 `conservative` 档仍为 `0.45 m`。

## 新增可观测量

`m20_mujoco_backend` 现在逐物理步识别
`warehouse_obstacle_*`/`warehouse_wall_*` 接触，并在
`/m20/sim/dynamics_state` 发布：

- 当前障碍接触数；
- 累计接触事件数；
- 累计峰值法向接触力；
- 接触 geom 对。

该统计排除了轮子与 `warehouse_ground` 的正常接触。逐步锁存避免 10 Hz
诊断漏掉毫秒级碰撞。

`m20_clearance_probe` 同时记录任务 Action、位姿、候选/安全速度、保护器状态、
首个阻挡样本、MuJoCo 接触和 B-spline 数量。探针按照实际航向计算前后双圆圆心
到固定障碍矩形的连续距离，并分别减去：

- 当轮测试的 SCAN 硬半径：`0.25 m`；
- 独立保护半径：`0.25 + 0.05 = 0.30 m`。

每次运行输出 `clearance_samples.csv` 和 `clearance_summary.json`。

## 单障碍基准

新增两个共享同一 PCD/PGM/MuJoCo 碰撞世界的隔离任务：

- `single_obstacle_double_bypass`：显式从障碍上、下两侧通过并回起点，用于验证
  任务流程和双侧可达性；
- `single_obstacle_direct_bypass`：起终点连线被障碍完全截断，不给中间点，由
  原生 SCAN 自主选择绕行侧，用于净空和安全前视整定。

第一轮双侧基准曾把障碍中心误当成楼层局部坐标，导致实体位于世界
`x=-2.5 m`。该轮证据保留并标记 `INVALID_RUN.md`，不纳入任何结论。生成器已改为
共享世界坐标 `x=-12.5 m`，回归测试同时断言最终矩形为
`[-13.1,-11.9] × [-1.0,1.0] m`。

## 参数分析

原 production `conservative` 足迹参数为：

```yaml
grid_map.double_cylinder_radius: 0.25
optimization.dist0: 0.20
footprint_radius: 0.25
footprint_offset: 0.18
safety_margin: 0.05
lookahead_sec: 1.00
```

单障碍直达任务中，SCAN 已绕到障碍上方，但保护器连续三次预测恒定转弯指令，
随后停车 `76.89 s` 并超时。三次触发均为 `PREDICTED_FOOTPRINT`，没有
`CURRENT_FOOTPRINT` 或 MuJoCo 障碍接触。

从 CSV 计算，预测停车后的最大一秒位移为 `0.156 m`。因此只在
`conservative` 档把 `lookahead_sec` 调为 `0.70 s`：

- 最高 `0.45 m/s` 时仍前视 `0.315 m`；
- 前视距离约为已测最坏停车位移的 `2.02` 倍；
- `0.25 m` 硬足迹、`0.05 m` 独立余量和栅格膨胀保持不变。

该修改减少恒转弯外推误差，不降低当前足迹碰撞保护。

## 结论

- `0.90 m + conservative` 是当前正式场景首个通过动态重复实验的最小默认档；
- `0.80 m + balanced` 单次虽通过，但最小连续保护净空只有 `2.3 mm`，只能作为
  实验边界，不能作为部署放行值；
- `0.75 m` 在调参前后都会由预测保护停车，且实体接触数始终为零；
- 调参后的单障碍直达绕行在 `28.24 s` 内完成，最小连续保护净空
  `0.129 m`，无保护停车、无实体接触；
- 本轮当时的 production 参数为 SCAN 硬半径 `0.25 m`、偏置 `0.18 m`、安全余量
  `0.05 m`、前视 `0.70 s`，场景障碍最小间隔继续保持 `0.90 m`。
- 后续航向跟随试验把 SCAN 硬半径改为 `0.30 m`、软距离改为 `0.15 m`，名义总包络
  仍为 `0.45 m`；最新结论见
  `docs/devlog/2026-07-30_heading_alignment_control.md`。

完整数值和验收判据见
`docs/test_reports/2026-07-30_dynamic_clearance_tuning.md`。
