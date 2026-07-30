# 0.9.0 SCAN-Planner 一致性与完整任务测试报告

日期：2026-07-27  
环境：Ubuntu 22.04、ROS 2 Humble、CycloneDDS loopback、RViz 运动学后端  
对照：当前工作空间 `src/third_party/SCAN-Planner`

## 自动测试

| 检查项 | 结果 |
| --- | --- |
| `planner_manager.cpp` 去注释/排版源码等价 | PASS |
| 原生 planner 共同参数逐键相等 | PASS |
| 原生 controller 参数逐键相等 | PASS |
| M20 几何/frame 允许覆盖白名单 | PASS |
| 单一 SCAN 核心、无适配包算法副本 | PASS |
| SCAN 原始可视话题全部存在 | PASS |
| Global Map/Sensor/Occupancy/Inflated/Path 显示属性 | PASS |
| Sensor Cloud 绑定实时 `/m20/sensing/sensor_cloud` | PASS |
| 自动任务默认 grid guide、局部 planner 为 SCAN | PASS |
| 原生/栅格增强 collision guard profile 隔离 | PASS |
| 碰撞预测速度与后端限速一致 | PASS |
| 三个改动包构建 | 3/3 PASS |
| 三个改动包 CTest/xUnit 汇总 | 166 tests，0 failures |
| 当前工作区 `colcon test-result --all` | 224 tests，0 failures |

RViz 配置使用 YAML 结构化回归，逐项比较本地第三方 `default.rviz`，不是只检查话题
字符串。GUI 桌面人工视觉复核状态仍为 `PENDING USER VIEW`，本报告不把 headless
配置测试写成截图验收。

## 统一两步入口冒烟测试

同一个 `inspection_mission_rviz.launch.py use_rviz:=false` 节点图中先发送与 RViz
`2D Goal Pose` 等价的 `/m20/navigation/scan_goal`：

```text
初始位置       (-42.000,  0.000)
手动目标       (-41.000, -1.000)
六秒后位置     (-41.004, -0.996)
```

不关闭或重启该节点图，随后运行：

```bash
ros2 run m20_warehouse_inspection m20_start_inspection
```

客户端连接到已经存在的 `/m20/mission/run`，依次报告：

```text
inspection started
[1/5] F1_A: STARTING, floor=F1, generation=1
[1/5] F1_A: NAVIGATING, floor=F1, generation=1
```

结果证明手动自由导航和自动巡检是同一个系统进程图的两种操作，不需要第二套 bringup。

## 纯原生完整任务对照

三次启动均使用：

```text
inspection_mission_rviz.launch.py
use_grid_route:=false
use_rviz:=false
run_acceptance:=true
acceptance_mode:=full
```

| 试验 | 保护配置 | 运行结果 |
| --- | --- | --- |
| native-1 | 0.50 m，未钳制预测速度 | F1_A 前碰撞保护截停 |
| native-2 | 0.30 m，未钳制预测速度 | 前进更远后截停 |
| native-3 | 0.30 m，预测速度钳制到后端限制 | 约 `(-37.5,-6.23)` 截停 |

共同证据：

- gateway 报告 `mode=native_scan`；
- 进程图中没有 `m20_grid_route_planner`；
- pause/resume 和 `MISSION_HOLD` 通过；
- SCAN 在截停期间仍持续产生成功的局部重规划；
- 独立 collision guard 报告 `COLLISION_STOP`；
- 三次任务状态都停留在 `NAVIGATING step=1/5 F1_A`，未伪报成功。

日志：

```text
log/runtime_0_9_native_full
log/runtime_0_9_native_full_guard_aligned
log/runtime_0_9_native_full_guard_clamped
```

结论：原生 profile 能复现 SCAN 自己的目标、重规划和可视化链，但不能承担当前随机
仓库的静态全局拓扑保证。不得通过关闭 collision guard 强行通过。

## 默认完整任务回归

最终回归使用自动任务默认值 `use_grid_route=true`：

```text
inspection_mission_rviz.launch.py
use_rviz:=false
run_acceptance:=true
acceptance_mode:=full
```

| 检查项 | 结果 |
| --- | --- |
| navigation gateway profile | `grid_route` |
| grid-route 保护半径 | 0.50 m |
| pause/resume 与安全零速 | PASS |
| F1_A | PASS |
| F1_B | PASS |
| E1 cabin 与 F1→F2 切图 | PASS |
| 新楼层 generation | F2 / 2 |
| F2_A | PASS |
| F2_TERMINAL `(8,0)` | PASS |
| 完成步数 | 5/5 |
| 最终位置误差 | 0.102 m |
| 验收标记 | `PHASE4_FULL_ACCEPTANCE_PASS` |

运行从 mission `STARTING` 到 PASS 约 257.8 s。A* 只发布顺序短目标；控制台可见每个
短目标都触发 `scan_planner_node` 的 `GEN_NEW_TRAJ/REPLAN_TRAJ`，生成新的 B 样条并由
原生 closed-loop controller 接收。因此完整任务不是用外部路径绕过 SCAN 控制。

日志：

```text
log/runtime_0_9_grid_full_final
```

PASS 标记在验收节点日志：

```text
PHASE4_FULL_ACCEPTANCE_PASS: 5 steps, pause/resume,
F2 elevator-lobby terminal, generation 2, final_error=0.102m
```

验收进程结束后人工 `Ctrl-C` 关闭常驻 launch；部分 Python 节点在 ROS 2 Humble
销毁句柄时记录 `KeyboardInterrupt/exit -2`。它发生在 PASS 之后，不属于任务运行
失败，完整回归的判据以验收节点完成时间为准。

## 最终判定

- SCAN 规划核心、原生参数和 SCAN RViz 核心图层：与本地第三方基线对齐。
- M20、平面双区域、PCD 切换、typed Action、安全层：明确的系统适配，不声称属于
  原项目。
- 自动五步仓库任务：使用外部静态全局引导保证可达性，SCAN 保持唯一局部轨迹规划和
  控制实现。
