# 2026-08-02 M20 边界恢复冷启动复验

## 结论

两个固定高风险用例各执行 3 次独立官方 SDK + MuJoCo 冷启动，共 `6/6` 导航成功；
没有实体障碍接触、后端故障、SCAN 膨胀区进入或后备路线带速换段。该结果证明本轮固定
边界回归通过，但不是对任意地图、姿态和动态扰动的绝对无碰撞保证。

汇总原始文件为
`docs/test_data/2026-08-02_boundary_recovery_final_matrix_summary.json`。

## 固定条件

| 项目 | 值 |
|---|---|
| 仿真后端 | 官方 M20 SDK + 官方 ONNX + MuJoCo 16 执行器模型 |
| 地图 | `dense_four_corner_system.yaml`，0.10 m occupancy |
| 正常规划 | vendor SCAN A* / rebound / B-spline / 原版可视化参数 |
| SCAN 参数 | 硬双圆半径 0.25 m，优化距离 0.20 m |
| native 执行保护 | 0.25 m 机身半径 + 0.05 m 安全余量 |
| 后备路线 | 0.60 m 栅格膨胀，0.90 m 子目标间距 |
| 段间停稳 | 0.04 m/s、0.08 rad/s、连续 0.30 s |
| 每个用例 | 全新 ROS/MuJoCo 进程，3 次独立冷启动 |

## 结果矩阵

| 用例 | 成功 | 实体接触 | 进入 SCAN 膨胀区 | 恢复覆盖 | 最差 SCAN 膨胀净空 | 时长中位数 / 最大值 |
|---|---:|---:|---:|---|---:|---:|
| `upper_boundary` | 3/3 | 0/3 | 0/3 | 后备 3/3；外壳脱离 2/3 | +0.116729 m | 94.732 / 105.176 s |
| `shared_origin_exit` | 3/3 | 0/3 | 0/3 | 边缘向内 3/3；后备 1/3 | +0.211961 m | 111.405 / 151.433 s |

六次运行的中间子目标停稳检查全部通过，最长 SCAN 膨胀区驻留时间均为 `0 s`，
MuJoCo 障碍接触事件峰值均为 `0`。`upper_boundary` 的一次运行未自然触发外壳分支，
但仍由正常保护/后备链完成，因此外壳分支动态覆盖是 2/3 而不是误写成 3/3。
`shared_origin_exit` 两次在边缘恢复后由原生 SCAN 完成，另一次进入后备路线。

## 接受条件

单次报告只有同时满足以下条件才记为 accepted：

- NavigateFloor Action 成功且最终误差在探针容差内；
- MuJoCo 障碍接触事件为零，接触对中没有机器人-障碍组合；
- 无 backend fault；
- 后备路线的每个非首子目标发布前已达到规定的连续停稳时间；
- 没有停稳违规或无界恢复；
- 样本和诊断数据完整。

## 复现

每次运行必须使用新的 0～232 ROS domain 和新的输出目录。例如：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=89
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI=file:///home/virdyn/robodog_nav_system/install/\
m20_warehouse_inspection/share/m20_warehouse_inspection/config/cyclonedds_local.xml

ros2 launch m20_warehouse_inspection boundary_recovery_mujoco.launch.py \
  case:=upper_boundary use_rviz:=false use_mujoco_viewer:=false \
  output_directory:=/tmp/m20_boundary_upper_run
```

将 `case` 改为 `shared_origin_exit` 可执行共享原点用例。多份结果使用：

```bash
python3 src/m20_warehouse_inspection/tools/summarize_boundary_recovery_runs.py \
  /path/to/upper/results /path/to/shared/results \
  --output /tmp/boundary_recovery_matrix_summary.json
```

聚合工具在任一报告未接受时返回非零，不应只人工观察最终位姿。

## 自动测试

受影响的四个包已构建通过。当前工作区 `colcon test-result --all --verbose` 汇总为
`477 tests`、`0 errors`、`0 failures`、`1 skipped`；跳过项是系统禁用的慢版
cppcheck。`m20_warehouse_inspection` 单包为 `176 tests`、`0 failures`、`1 skipped`。

## 历史失败证据与放行边界

开发过程中保留了 `boundary_recovery_cold_starts`、`postfix_cold_starts`、
`final_cold_starts` 等修复前/中间数据，用来证明问题确实被复现。最终 6 次矩阵只引用：

- `boundary_recovery_segment_hold_cold_starts/upper_boundary/run_01..03`；
- `boundary_recovery_forward_gate_cold_starts/shared_origin_exit/run_01..03`。

一次使用 `ROS_DOMAIN_ID=233` 的启动因 CycloneDDS UDP 端口范围无效而未形成有效报告，
属于基础设施失败，已排除并用合法 domain 89 重跑。当前放行的是两组固定边界回归、
受约束外壳/地图边缘恢复和显式段间保持。后续新增地图、提高速度、修改 M20 策略或
实机定位噪声模型后，必须重新运行该矩阵和完整 11 步任务。

