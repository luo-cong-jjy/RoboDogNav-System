# 2026-08-02 默认双区域 MuJoCo 11 步验收报告

## 结论

默认主系统使用官方 M20 SDK + ONNX 策略 + MuJoCo 动力学完成
`dense_four_corner_patrol` 全部 `11/11` 步。任务从 F1 起点出发，覆盖两块区域四角，
在共享原点完成两次 active PCD/occupancy 切换，最后返回整个流程起点。最终 Action
状态为 `COMPLETED`，没有 `FAULT_HOLD` 或任务超时。

本轮还真实复现了先前 F2 左上失败点：原生 SCAN 的预测碰撞恢复耗尽后，系统进入 3 段
后备路线；第 1 段到达后等待实测速度稳定停止，再下发第 2 段，随后完成第 9 点。
因此 stop-to-stop 修复具有真实动力学覆盖，而不是只通过静态测试。

## 固定条件

| 项目 | 值 |
|---|---|
| ROS domain | 210 |
| 主启动 | `inspection_mission_mujoco.launch.py use_rviz:=false use_mujoco_viewer:=false` |
| 任务 | `dense_four_corner_patrol` |
| 场景 | 双 40 m 平面区域，默认最小独立障碍间距 0.90 m |
| 规划主链 | vendor SCAN A* / rebound / B-spline / 原版可视化参数 |
| M20 后端 | 官方 16 执行器 MJCF、官方 `policy.onnx`、`CmdVelInterface` |
| 正常/后备 | `use_grid_route=false`，碰撞恢复耗尽时临时启用后备路线 |
| 后备膨胀 | 0.60 m |
| 段间停稳 | 0.04 m/s、0.08 rad/s、连续 0.30 s，超时 6 s |
| 实测速度 PI | 默认关闭 |

## 任务结果

| 步骤 | 结果 | 关键观测 |
|---:|---|---|
| 1～4 | 通过 | F1 左下→右下→右上→左上，正常使用原生 SCAN |
| 5 | 通过 | 到共享原点后 hold，F1 generation 1→F2 generation 2，等待 3 帧新点云 |
| 6～8 | 通过 | F2 左下→右下→右上；本次未强制触发后备路线 |
| 9 | 通过 | F2 左上发生预测碰撞恢复耗尽，3 段后备路线真实接管 |
| 10 | 通过 | 后备路线到原点约 `(0.105, 0.124)` 后才切换至 F1 generation 3 |
| 11 | 通过 | 返回目标 `(-37,0)`，任务报告 `COMPLETED` |

任务时间戳从 `1785652489.170` 到 `1785653568.686`，总耗时约
`1079.5 s`（17 min 59.5 s）。这是完整物理与官方策略实时运行时间，不是质点加速回放。

## 第 9 步动态证据

1. `(32.8,16.8)` 附近 collision guard 报告
   `RECOVERY_BUDGET_EXHAUSTED; reason=TIME_LIMIT`；
2. 网关 reset SCAN，后备路线发布 35 个显示位姿、28.94 m、3 个执行子目标；
3. 子目标 1 为 `(32.05,16.45)`；
4. `1785653272.733` 报告子目标到达并开始等待实测停止；
5. MuJoCo parking brake 在 `1785653274.476` 进入 STOPPED；
6. `1785653274.954` 才发布子目标 2 `(9.85,16.05)`；
7. 第 9 点最终进入 DWELLING，而非此前的 `FAULT_HOLD`。

发布间隔约 2.22 s，证明下一段不是在 0.30 m 接受半径触发瞬间带速下发。

## 换层与返回

- 第一次换层顺序：到原点→floor-switch hold→F2 generation 2→SCAN reset→3 帧新点云；
- 第二次换层前，F2 后备路线完成且位姿进入原点容差，随后才切到 F1 generation 3；
- 第 11 步在 active map 边界因后圆预测越界安全触发一次后备直线路线，随后完成返回；
- 全流程没有传送机器狗位姿，切换的是 active PCD/occupancy 和 generation。

## 自动回归

受影响范围执行：

```text
colcon test --packages-select
  m20_scan_navigation m20_scan_planner m20_warehouse_inspection
  m20_inspection_core m20_locomotion_control m20_mujoco_backend
  m20_warehouse_sim
```

第一次 `m20_scan_planner` launch test 因受限环境不能写 `~/.ros/log` 失败；将
`ROS_LOG_DIR` 指向 `/tmp/m20_colcon_ros_logs_20260802` 后两个 launch test 均通过。
最终工作区汇总为 `479 tests`、`0 errors`、`0 failures`、`1 skipped`。跳过项是系统
版本禁用的慢版 cppcheck，不是功能测试失败。

## 放行范围

放行碰撞后备路线的共享门洞处理、边界直线倒车约束、子目标停滞检测和 stop-to-stop
实测速停稳。正常 SCAN 规划参数与可视化保持 vendor 基线。延长短倒车首段已通过单元
测试，但本次完整任务没有自然触发该特定分支，后续可在固定姿态边界用例中补充重复
动态覆盖。
