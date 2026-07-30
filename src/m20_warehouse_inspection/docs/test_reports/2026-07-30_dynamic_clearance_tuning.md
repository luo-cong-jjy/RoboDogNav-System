# 动态净空与单障碍保护测试报告

日期：2026-07-30

## 测试栈

- ROS 2 Humble；
- 原生 SCAN-Planner 轨迹链，`use_grid_route=false`；
- 官方 M20 MJCF、官方 SDK/ONNX 运控和 MuJoCo；
- 0.05 m 标准门洞/单障碍占据图；
- 前后双圆：半径 `0.25 m`、偏置 `±0.18 m`；
- 独立安全余量：`0.05 m`；
- 运行证据：工作空间
  `artifacts/clearance_dynamics_2026-07-30/<case>/`。

成功运行中 `collision_stop_sample_count=1` 是探针启动时收到的 latched
`NOT_READY` 样本；运行期碰撞事件以 `guard_events` 为准。

## 结果

| 场景 | 参数 | 结果 | 用时 | 最小 guard 净空 | 预测/当前停车 | 障碍接触 |
|---|---|---:|---:|---:|---:|---:|
| 0.75 m 门洞 | tight，1.00 s | 拒绝/超时 | 100.02 s | 0.0119 m | 1 / 0 | 0 |
| 0.80 m 门洞 | balanced，1.00 s | 通过 | 30.96 s | 0.0023 m | 0 / 0 | 0 |
| 0.90 m 门洞 R1 | conservative，1.00 s | 通过 | 30.96 s | 0.0523 m | 0 / 0 | 0 |
| 0.90 m 门洞 R2 | conservative，1.00 s | 通过 | 30.96 s | 0.0523 m | 0 / 0 | 0 |
| 0.90 m 门洞 R3 | conservative，1.00 s | 通过 | 30.96 s | 0.0523 m | 0 / 0 | 0 |
| 单障碍直达 | conservative，1.00 s | 拒绝/超时 | 110.02 s | 0.1094 m | 3 / 0 | 0 |
| 单障碍直达 | conservative，0.70 s | 通过 | 28.24 s | 0.1293 m | 0 / 0 | 0 |
| 0.75 m 门洞 | conservative，0.70 s | 拒绝/超时 | 70.02 s | 0.0522 m | 1 / 0 | 0 |
| 0.90 m 门洞 | conservative，0.70 s | 通过 | 30.96 s | 0.0523 m | 0 / 0 | 0 |

补充流程测试：修正坐标后的单障碍双侧绕行 6/6 步完成，最终回起点误差
`0.147 m`，无停车、无实体接触。由于显式航点离障碍较远，它只用于双侧流程验证，
不参与极限净空结论。

## 验收判断

### 0.90 m 默认值

三次独立冷启动均通过；最小保护净空范围为
`0.05226～0.05232 m`，重复运行离散小于 `0.1 mm`。调参后正向回归保持相同净空，
说明前视调整没有改变静态足迹或路径几何。

### 0.80 m 边界

单次动态通过且无实体碰撞，但只有 `2.3 mm` 连续保护余量。该宽度无法覆盖定位、
点云、栅格量化和实物跟踪误差，不升级为正式最小间隔。

### 0.75 m 安全拒绝

调参后在通道中部触发
`PREDICTED_FOOTPRINT; t=0.65 s`，任务进入 `FAULT_HOLD`。全过程：

- 无 `CURRENT_FOOTPRINT`；
- 无 MuJoCo 障碍接触；
- 最小连续 guard 净空仍为 `0.0522 m`。

因此 0.70 s 前视不会把不可接受的窄道错误放行。

### 单障碍

1.00 s 恒转弯外推产生三次预测停车和 `76.89 s` 指令静止期。前视调为
0.70 s 后，同一地图、同一目标、同一足迹在 `28.24 s` 完成：

- 最小 SCAN 硬净空：`0.1793 m`；
- 最小独立保护净空：`0.1293 m`；
- 预测/当前停车：`0/0`；
- 障碍接触事件和峰值力：`0 / 0 N`。

## 最终配置

以下是本轮动态净空测试结束时的配置。后续航向跟随测试把 SCAN 硬半径调整为
`0.30 m`、`optimization.dist0` 调整为 `0.15 m`，总单侧名义包络仍为
`0.45 m`。当前正式值与动态依据见
`2026-07-30_heading_alignment_control.md`。

```yaml
scan_planner_node:
  ros__parameters:
    grid_map.double_cylinder_radius: 0.25
    optimization.dist0: 0.20

m20_grid_route_planner:
  ros__parameters:
    inflation_radius: 0.45

m20_collision_guard:
  ros__parameters:
    footprint_radius: 0.25
    footprint_offset: 0.18
    safety_margin: 0.05
    lookahead_sec: 0.70
```

结论：保留 `0.90 m` 场景最小障碍间隔和 `conservative` 默认档，不减小机身、
安全余量或可选栅格路线膨胀。
