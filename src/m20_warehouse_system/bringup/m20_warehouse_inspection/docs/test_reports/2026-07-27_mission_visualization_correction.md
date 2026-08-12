# 0.8.0 任务与显示校正测试报告

日期：2026-07-27  
环境：Ubuntu 22.04、ROS 2 Humble、CycloneDDS loopback、RViz 运动学后端

## 自动测试

| 项目 | 结果 |
| --- | --- |
| terminal 配置/解析 | PASS |
| 默认五步名称与 `(8,0,pi)` 终点 | PASS |
| 自动任务 grid-route 默认值 | PASS |
| inactive-only RViz 话题 | PASS |
| 默认 RViz 不再叠加 all-floors cloud | PASS |
| PGM 四条围栏边占据 | PASS |
| `m20_inspection_core` CTest | 9/9 PASS |
| `m20_warehouse_inspection` CTest | 13/13 PASS |
| 工作区汇总 | 215 tests，0 failures |

## 完整任务运行

| 检查项 | 结果 |
| --- | --- |
| map/sensing/navigation ready | PASS |
| navigation gateway profile | `grid_route` |
| pause/resume 安全保持 | PASS |
| F1_A | PASS |
| F1_B | PASS |
| E1 F1→F2 | PASS |
| F2_A | PASS |
| F2_TERMINAL | PASS |
| 完成步数 | 5/5 |
| 最终 floor/generation | F2 / 2 |
| 最终 `(8,0)` 误差 | 0.123 m |
| 第 5 步路线 | 16.23 m / 20 子目标 |
| 验收标记 | `PHASE4_FULL_ACCEPTANCE_PASS` |

第 5 步从 `NAVIGATING` 到 `STEP_COMPLETE` 约 49.05 s，期间子目标从 1/20 连续推进到
20/20，不存在“长时间静止且无规划进度”的现象。

## GUI 状态

本报告的端到端验收使用 `use_rviz:=false`。默认 RViz 配置已改为黄色 active floor 与
蓝灰色 inactive floor 的互斥点云；GUI 人工视觉复核状态为 `PENDING USER VIEW`，不把
headless 配置检查误记为桌面截图验收。

局部滑动窗口中的半透明红色 `SCAN Inflated Occupancy` 继续保留；它与黄色
`SCAN Occupancy` 重叠后出现粉红色属于预期的安全膨胀显示。
