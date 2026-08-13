# 2026-08-02 M20 净空与后向目标复验

## 结论

固定官方 M20 SDK + 官方 ONNX + MuJoCo 动态复验中：

- 长距离 `upper_boundary` 在原生轨迹触发有界保护后切换到软净空路线，Action 成功；
- `rear_corridor` 的目标位于当前朝向正后方 1 m，控制器选择 REVERSE，不执行掉头，
  Action 成功；
- `narrow_corridor_entry` 复现用户的同一起点/目标，主动净空路线避免进入不可转弯
  口袋并成功到达；
- 三次接受运行均没有机器人-障碍接触、backend fault 或进入 SCAN 膨胀区。

这是对两类固定回归的放行证据，不代表任意地图、定位误差和实机地面条件下的绝对保证。
关键结果快照保存在
`docs/test_data/2026-08-02_clearance_heading_policy/acceptance_summary.json`。

## 静态测试

相关测试覆盖：

- 软代价使有多条路线时的最小栅格净空从 1 cell 提升到至少 3 cells；
- 路径简化和 SCAN 子目标压缩保持所选软净空；
- 只有一条自由通道时，软代价不把它关闭；
- 后向角阈值、最小距离和 position-only 配置契约；
- 最终目标接受半径与 SCAN 的 0.20 m 阈值一致。

本轮聚焦执行结果为 `38 passed`。包级完整结果记录在下方“最终回归”。

## 动态结果

| 用例 | Action | 时长 | 最终误差 | 最小实体净空 | 最小 SCAN 膨胀净空 | 接触 | 进入膨胀区 |
|---|---:|---:|---:|---:|---:|---:|---:|
| `upper_boundary` | 成功 | 96.800 s | 0.091 m | 0.397 m | +0.147 m | 0 | 0 samples |
| `rear_corridor` | 成功 | 4.333 s | 0.184 m | 1.995 m | +1.745 m | 0 | 0 samples |
| `narrow_corridor_entry` | 成功 | 35.474 s | 0.129 m | 0.636 m | +0.386 m | 0 | 0 samples |

`upper_boundary` 先按原生 SCAN 执行并保留有界后退，保护预算耗尽后等待 CLEAR，再使用
3 个 SCAN 子目标完成路线；2 次中间换段均满足停稳约束。`rear_corridor` 从
`(-36, 0, yaw≈0)` 到 `(-37, 0)`，最终 yaw 为 `-0.000588 rad`，说明机身保持原朝向
倒行，而不是旋转约 pi。运行日志明确报告 `Trajectory tracking direction: REVERSE`。

第一次后向试验还复现了 `0.15 m` 适配器阈值与 `0.20 m` SCAN 阈值不一致导致的假
`SUBGOAL_STALLED`；统一为 `0.20 m` 后同一冷启动用例通过。该失败不计入接受结果。

用户实时失败从 `(-37,0)` 向 `(-30.62,3.18)` 使用延迟接管策略，最终停在
`(-34.379,2.300)` 并报告 `COLLISION_REPLAN_EXHAUSTED`。主动路线对照使用 6 个 SCAN
子目标，5 次中间停稳违规为 0，保护/恢复事件为 0，证明问题是接管时机而不是通道被
硬膨胀完全封闭。该证据促使完整系统默认改为 `use_grid_route=true`。

## 最终回归

完成开发后执行：

```bash
colcon test --packages-select m20_scan_navigation m20_warehouse_inspection
colcon test-result --all --verbose
```

最终结果：`m20_scan_navigation` 为 `41 tests`、`0 failures`；
`m20_warehouse_inspection` 为 `196 tests`、`0 failures`、`1 skipped`。跳过项是系统因
已知性能问题禁用的慢版 cppcheck，不是功能测试失败。`git diff --check` 与 JSON 格式
校验同时通过。

## 复现后向用例

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash

ros2 launch m20_warehouse_inspection boundary_recovery_mujoco.launch.py \
  case:=rear_corridor use_rviz:=false use_mujoco_viewer:=false \
  output_directory:=/tmp/m20_rear_corridor
```

正常主入口不需要测试参数；该固定用例只用于回归。
