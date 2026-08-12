# 候选速度与轨迹冻结动态回归报告

日期：2026-07-30

## 用例

- 场景：正式 `dense_four_corner`，最小障碍物本体间距 0.90 m；
- 起点：`(-37.0, 0.0)`；
- 目标：F1 左下巡检点 `(-36.0, -16.0)`；
- 导航：原生 SCAN rebound/B 样条；
- clearance：`conservative`；
- 动态后端：
  1. RViz 平面运动学；
  2. MuJoCo 16 关节动力学 + 官方 M20 SDK/ONNX；
- RViz 和 MuJoCo 均使用同一 candidate、安全、双圆碰撞保护和冻结代码。

该目标覆盖历史卡住位置附近。历史故障位姿约为
`(-35.011, -8.423)`，新运行均连续越过该区段。

## 结果

| 指标 | RViz | MuJoCo + 官方 SDK |
|---|---:|---:|
| 到达目标 | PASS | PASS |
| 最终位置 | `(-35.996,-15.836)` | `(-36.044,-15.959)` |
| 最终 XY 误差 | 0.164 m | 0.060 m |
| 接收 B 样条 | 21 | 29 |
| `A-star failed` | 0 | 0 |
| `First three control points are in obstacles` | 0 | 0 |
| 运行期 collision guard STOP | 0 | 0 |
| 最终 collision guard | CLEAR | CLEAR |
| 最终 safety state | NAVIGATION | NAVIGATION |

启动阶段 safety supervisor 在碰撞保护尚未就绪时会按设计进入 fail-closed
`COLLISION_STOP`；表中运行期停车不计该启动门控。

## 冻结恢复观测

测试主机出现约 1.5～1.9 s 的调度间隙，触发 `ODOM_STALE` 或
`COMMAND_TIMEOUT`。新链路表现为：

1. safety 发布 execution hold；
2. closed-loop controller 报告 `External trajectory hold asserted`；
3. SDK 输出立即归零并启用停车制动；
4. SCAN 下一次恢复规划打印起点速度 `0 0 0`；
5. 新轨迹发布后 hold 解除并继续前进；
6. 两个后端均最终到达目标。

MuJoCo 在导航期间经历两次上述恢复，没有转化为碰撞停止、旧轨迹导数继承或局部
A* 日志风暴。

## 静态与包级验证

```text
colcon build:
  m20_locomotion_control
  m20_inspection_core
  m20_scan_planner
  m20_scan_navigation
  m20_warehouse_inspection
  PASS

colcon test-result --verbose:
  363 tests
  0 errors
  0 failures
  0 skipped
```

双圆策略专项测试还验证：机器人中心仍在自由格时，前圆进入膨胀障碍会在当前样本立即
报告 `front/OCCUPIED`。

## 结论

本次修复对用户报告的故障链形成了直接回归：同一目标、同一问题区段、两种后端均
完成，三个核心错误计数均归零。当前修改可作为默认链路保留。

仍需后续执行完整 11 步 MuJoCo 巡检和多轮路线挑战，统计长期碰撞保持次数、重规划
延迟和最小动态净空；单目标通过不等同于整套场景耐久验收。
