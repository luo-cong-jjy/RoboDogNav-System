# 2026-08-01 M20 倒车与速度反馈再资格验证报告

## 结论

直线倒车方向选择已经通过官方 M20 SDK + MuJoCo 六目标冷启动验证：`6/6`，两个
原航向回程均稳定使用 `REVERSE`，曲线与 90° 路段保持 `FORWARD`，没有碰撞停车、
恢复、恢复预算耗尽、实体接触或 backend fault。

近似直线速度反馈在开阔场景改善总耗时、平均终点误差和平均横向误差，但单障碍两次
冷启动的最小硬余量差异达到 `86.3 mm`。它尚未达到生产默认所需的障碍安全重复性，
因此主配置继续使用 `velocity_feedback_enabled: false`。

## 固定验证条件

| 项目 | 值 |
|---|---|
| 动力学与模型 | 官方 M20 16 执行器 MJCF + MuJoCo |
| 底层运控 | 官方 `policy.onnx` + `CmdVelInterface` |
| 导航 | 原生 SCAN A* / B-spline / 原重规划参数 |
| SCAN 硬半径 / 优化距离 | `0.25 m / 0.20 m` |
| 独立 guard | 前后双圆 `0.25 + 0.05 = 0.30 m` |
| 速度反馈 PI | 线速度 `0.30/0.08`，角速度 `0.20/0.05` |
| 反馈修正上限 | `0.10 m/s / 0.12 rad/s` |
| 最终实验门控 | `WHEEL_CRUISE` 且 `|wz_ref| <= 0.08 rad/s` |

每组均重新启动 MuJoCo 和官方策略，不复用前一任务的动力学状态。

## 六目标 A/B

| 版本 | 成功 | 总用时 | 平均终点误差 | 平均最大横向误差 | 全局最大横向误差 | 线速度 RMS | 角速度 RMS |
|---|---:|---:|---:|---:|---:|---:|---:|
| 反馈关闭 | 6/6 | `97.695 s` | `0.1284 m` | `0.2586 m` | `0.7641 m` | `0.14882 m/s` | `0.09182 rad/s` |
| 全程反馈 | 6/6 | `96.708 s` | `0.0937 m` | `0.3040 m` | `0.8891 m` | `0.14580 m/s` | `0.08390 rad/s` |
| 仅巡航 | 6/6 | `96.987 s` | `0.1239 m` | `0.2725 m` | `0.8517 m` | `0.14829 m/s` | `0.08702 rad/s` |
| 近似直线巡航 | 6/6 | `92.767 s` | `0.1011 m` | `0.2506 m` | `0.7725 m` | `0.14521 m/s` | `0.08792 rad/s` |

相对反馈关闭，最终门控版本：

- 总用时减少 `5.0%`；
- 平均终点误差减少 `21.3%`；
- 平均最大横向误差减少 `3.1%`；
- 全局最大横向误差增加 `8.4 mm`；
- 反馈 active 样本比例为 `66.9%`。

四组均无碰撞、恢复、预算耗尽、实体接触和 backend fault。全程反馈虽然提高速度跟踪，
但放大转弯横向偏差；门控是必要的，不应删除。

### 最终门控版本逐目标结果

| 路段 | 用时 | 终点误差 | 最大横向误差 | 跟踪方向 |
|---|---:|---:|---:|---|
| 前进 2.5 m | `9.64 s` | `0.073 m` | `0.018 m` | FORWARD |
| 原航向回程 2.5 m | `12.89 s` | `0.151 m` | `0.043 m` | REVERSE |
| 右转 90° 后 6 m | `20.78 s` | `0.120 m` | `0.772 m` | FORWARD |
| 左转 90° 后 3 m | `11.19 s` | `0.055 m` | `0.337 m` | FORWARD |
| 原航向回程 3 m | `15.79 s` | `0.151 m` | `0.021 m` | REVERSE |
| 左转 90° 返回 6 m | `22.49 s` | `0.057 m` | `0.313 m` | FORWARD |

每个目标内部均未发生方向切换，说明 B-spline 级直线判定消除了端点切向抖动。

## 单障碍复验

| 版本 | 成功 | 用时 | 终点误差 | 最小硬余量 | 最小 guard 余量 | 预测恢复 | 实体接触 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 反馈关闭 | 是 | `30.449 s` | `0.0907 m` | `0.2359 m` | `0.1859 m` | 2 | 0 |
| 近似直线反馈，第 1 次 | 是 | `32.864 s` | `0.1797 m` | `0.1645 m` | `0.1145 m` | 1 | 0 |
| 近似直线反馈，第 2 次 | 是 | `30.243 s` | `0.0770 m` | `0.2508 m` | `0.2008 m` | 1 | 0 |

三次都没有当前足迹停车、恢复预算耗尽或实体接触。两次反馈复验分别从障碍物两侧绕行，
反映 SCAN 局部重规划/同伦选择的冷启动差异。第二次结果不能抵消第一次安全余量回退，
生产放行必须关注最差样本而不是均值。

## 自动回归

本报告提交前执行以下范围：

```text
colcon build --symlink-install --packages-select
  m20_scan_planner m20_scan_navigation
  m20_locomotion_control m20_warehouse_inspection

colcon test --packages-select
  m20_scan_planner m20_scan_navigation
  m20_locomotion_control m20_warehouse_inspection
```

结果：四个受影响包构建和测试均通过；工作区累计 `467 tests`、`0 errors`、
`0 failures`、`1 skipped`。跳过项是当前系统版本禁用的慢版 cppcheck，不是功能
测试失败。

## 原始数据

- 六目标反馈关闭：
  `docs/test_data/2026-08-01_bidirectional_velocity_feedback_ab/open_loop_latched/`
- 六目标全程反馈：
  `docs/test_data/2026-08-01_bidirectional_velocity_feedback_ab/closed_loop_latched/`
- 六目标仅巡航反馈：
  `docs/test_data/2026-08-01_bidirectional_velocity_feedback_ab/closed_loop_cruise_only/`
- 六目标近似直线反馈：
  `docs/test_data/2026-08-01_bidirectional_velocity_feedback_ab/closed_loop_straight_only/`
- 单障碍反馈关闭：
  `docs/test_data/2026-08-01_bidirectional_velocity_feedback_ab/single_obstacle_open/`
- 单障碍反馈第 1/2 次：
  `docs/test_data/2026-08-01_bidirectional_velocity_feedback_ab/single_obstacle_straight_only/`
  和 `single_obstacle_straight_only_repeat/`

## 放行范围与下一步

放行“新 B-spline 级近似直线倒车判定与倒车航向锁存”。速度反馈保留显式实验入口，
默认关闭。下一步回到主系统执行默认 `0.90 m` 双区域完整巡检回归，验证导航、原点
切层、F2 全覆盖和返回系统起点的端到端链路；若以后推广速度反馈，需要增加明确的
邻障碍门控信号和至少多次成对冷启动最差值验收。
