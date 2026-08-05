# 2026-08-02 M20 连续交接验证记录

## 当前结论

固定 `narrow_corridor_entry` 的离线路线回放保持原全局几何：

| 指标 | 结果 |
|---|---:|
| 起点 / 目标 | `(-37, 0)` / `(-30.62, 3.18)` |
| 全局路线点 | 14 |
| 压缩 SCAN 子目标 | 6 |
| 连续交接 | 5 |
| 强制中间停车 | 0 |
| 最终目标停车 | 保留 |

六个逻辑子目标仍为 `(-31.95,0.55)`、`(-30.85,0.75)`、
`(-30.55,1.05)`、`(-30.45,1.55)`、`(-30.45,2.75)`、
`(-30.62,3.18)`；即本轮没有为了视觉丝滑而改变已经通过的 A* 路径。

单元/契约测试覆盖开阔缓弯连续、超过 0.70 rad 急弯停车、靠近硬障碍的缓弯停车、
终点必停、真实 dense 场景固定路线结果，以及动态探针区分连续交接和停稳交接。
针对性结果为 `42 passed`。完整包级回归的 23 个 CTest 套件全部成功；工作区已有结果
汇总为 `504 tests, 0 errors, 0 failures, 1 skipped`，跳过项仍是已知慢版 cppcheck。

## 动态复验状态

本轮尝试启动隔离 DDS 域中的无界面 MuJoCo 固定用例时，执行环境在进程创建前因外部
执行额度限制拒绝命令；没有产生新的 MuJoCo 运行结果，也不是仿真程序失败。因此本
记录不把离线结果冒充动态放行证据。上一版同一路径的 stop-to-stop 动态基线仍为
35.474 s、最终误差 0.129 m、零接触、零膨胀区采样。

本机动态复验命令：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=38
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI=file:///home/virdyn/robodog_nav_system/install/m20_warehouse_inspection/share/m20_warehouse_inspection/config/cyclonedds_local.xml

ros2 launch m20_warehouse_inspection boundary_recovery_mujoco.launch.py \
  case:=narrow_corridor_entry use_grid_route:=true \
  use_rviz:=false use_mujoco_viewer:=false timeout_sec:=140.0 \
  output_directory:=/tmp/m20_continuous_handoff_20260802
```

放行标准：Action 成功、接触为 0、膨胀区采样为 0、backend fault 为 0；报告中
`continuous_handoff_count` 应大于 0，且任何保留下来的
`intermediate_stop_violation_count` 必须为 0。
