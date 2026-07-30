# M20 航向跟随与预测停车恢复测试报告

日期：2026-07-30

## 验收目标

1. 机器人在直线和窄通道中不因机身航向落后而切向障碍；
2. 预测保护停车不能允许平移穿入膨胀区；
3. 预测停车可在纯旋转安全时恢复，当前足迹碰撞仍必须硬停车；
4. 不修改原版 SCAN 规划和可视化算法；
5. MuJoCo 障碍接触事件必须为零。

## 自动测试

```text
125 passed in 9.80s
```

覆盖内容包括航向迟滞、最短保持、纯偏航清零、转向限速、保护区旋转正反向选择、
当前足迹禁止恢复、launch 参数接线、默认包络以及原有任务/切图回归。

## 动态试验

所有有效试验使用官方 M20 MJCF、官方 ONNX 运控、MuJoCo 动力学和原生 SCAN
B-spline 链。`collision_stop_sample_count=1` 且 `guard_events=[]` 的运行是探针启动时
收到的 latched `NOT_READY`，不计为运行期碰撞停车。

| 试验 | 关键设置 | 结果 | 用时 | 最小 guard 净空 | 预测/当前停车 | 实体障碍接触 |
|---|---|---:|---:|---:|---:|---:|
| 六目标初版航向对正 | 0.35/0.15 rad；旧 SDK 转向参数 | 6/6 通过 | 各 9.36～24.49 s | 非净空场景 | 0 / 0 | 0 |
| 单障碍初版 | 0.35/0.15 rad | 通过 | 71.42 s | -0.0037 m | 1 / 0 | 0 |
| 单障碍无恢复 | 0.55 rad；`turn_max_forward=0.05` | 预测停车死锁 | 超时 | 正净空 | >0 / 0 | 0 |
| 0.90 m 通道最终跟踪实验 | 硬半径 0.30 m；gain 1.20；爬行 0.05 | 通过 | 31.21 s | 0.0759 m | 0 / 0 | 0 |
| 单障碍纯转向边界实验 | 硬半径 0.30 m；爬行 0 | 通过 | 29.10 s | 0.2183 m | 0 / 0 | 0 |
| 六目标纯转向边界实验 | 爬行 0；旧退出阈值 0.12 rad | 5/6，否决 | 第六段 90 s 超时 | 无停车 | 0 / 0 | 0 |

窄通道通过证据：

```text
artifacts/navigation_motion/
  2026-07-30_heading_alignment_narrow_090_tracking_v1/
    clearance_summary.json
```

该次结果：

- 最终误差 `0.1536 m`；
- 最小 SCAN 硬足迹净空 `0.1259 m`；
- 最小独立保护净空 `0.0759 m`；
- 最长指令静止 `0.305 s`；
- 20 条 B-spline；
- 预测/当前停车 `0/0`；
- 障碍接触事件和峰值力 `0 / 0 N`。

与此前同一 `0.90 m` 基准的约 `0.0523 m` guard 净空相比，航向修正后增加约
`23.6 mm`，说明改进来自跟踪居中，而不是缩小保护区。

## 否决实验

### 增大 SCAN 硬半径

`0.32 m` 和 `0.35 m` 均在 `0.05 m` GridMap 中量化到下一圈占据单元，使 `0.90 m`
通道失去可行 A-star 路径。因此硬半径冻结为 `0.30 m`，其余 `0.15 m` 留在优化器
软距离中。

### 转向时完全禁止前进

`turn_max_forward=0` 的单障碍结果最好，但六目标自由导航第六段超时，且第二段路径长度
比达到 `2.72`、运动模式切换 60 次。该设置被否决。最终采用 `0.05 m/s` 低速爬行，
并把转向偏航退出阈值从 `0.12` 放宽到 `0.20 rad`，减少接近目标时长期滞留转向模式。

否决数据保留在：

```text
artifacts/navigation_motion/
  2026-07-30_heading_alignment_obstacle_turn_first/
  2026-07-30_heading_alignment_final/
```

## 当前最终参数

```yaml
controller:
  heading_slowdown_threshold: 0.15
  heading_error_threshold: 0.55
  heading_error_resume_threshold: 0.20
  heading_alignment_min_hold_sec: 0.40

navigation_adapter:
  course_yaw_gain: 1.20
  cruise_yaw_deadband: 0.02
  turn_max_forward: 0.05
  turn_yaw_exit: 0.20
  turn_min_hold_sec: 0.50

scan:
  grid_map.double_cylinder_radius: 0.30
  optimization.dist0: 0.15

collision_guard:
  footprint_radius: 0.25
  safety_margin: 0.05
  lookahead_sec: 0.70
```

## 放行状态

代码、参数接线和 125 项自动测试已通过；`0.90 m` 窄通道在与最终参数主体一致的
`turn_max_forward=0.05` 配置下动态通过，且实体接触为零。最后一次六目标试验用于否决
`turn_max_forward=0`，不是当前默认配置。

当前默认组合把爬行恢复为 `0.05 m/s`、将 `turn_yaw_exit` 改为 `0.20 rad` 后，整套
六目标冷启动复验曾因本机 ROS 进程执行授权通道中断而未完成。该项必须保留为最终动态
确认项，不能用静态测试或被否决的纯转向数据冒充已通过。

