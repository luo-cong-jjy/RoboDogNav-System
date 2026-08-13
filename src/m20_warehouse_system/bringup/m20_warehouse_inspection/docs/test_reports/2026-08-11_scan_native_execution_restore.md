# 2026-08-11 原生 SCAN 执行链回归报告

## 构建与源码测试

- 受影响的 7 个包完成 `colcon build --symlink-install`。
- 定向契约/策略测试：57/57 通过；五个受影响功能包的最终完整源码测试：
  211/211 通过。
- `m20_scan_planner/src/planner_manager.cpp` 去注释和空白后与 vendored
  `SCAN-Planner` 完全一致。
- 主 planner YAML 的参数键和值与 vendored 配置完全一致；未保留 5 Hz 可视化
  降频覆盖。
- 默认 launch 参数为 `execution_profile:=scan_native`；真机 launch 静态固定
  `execution_profile:=m20_safe`。

## RViz 运动学实跑

隔离域：`ROS_DOMAIN_ID=181`，关闭 RViz GUI，启动日常主入口。

- 节点图包含 SCAN、controller、map、mission、floor switch 和 safety gate；
  不包含 `m20_navigation_adapter`、`m20_collision_guard`。
- 起点：`(-37.000, 0.000)`。
- 目标：`(-35.000, 0.000)`。
- 结果：SCAN 两次规划均 `final_plan_success=1`，终态 `WAIT_TARGET`。
- 最终位置：`(-35.0264, 0.0000)`，平面误差约 `0.0264 m`。
- 未出现 `PREDICTED_FOOTPRINT`、`RECOVERY_BUDGET_EXHAUSTED`、
  `COLLISION_STOP` 或 A-star inside-obstacle 错误。

## 完整 MuJoCo 一键实跑

隔离域：`ROS_DOMAIN_ID=182`，执行
`inspection_mission_mujoco.launch.py use_rviz:=false use_mujoco_viewer:=false`。

- 官方 SDK 正常完成 `idle -> standup -> rl_control`。
- MuJoCo 发布 backend ready，M20 站立稳定。
- 默认节点图无 navigation adapter、collision guard 和 candidate 速度话题。
- 起点：`(-37.000, 0.003)`。
- 目标：`(-35.500, 0.000)`。
- 两次 SCAN 规划均 `final_plan_success=1`，终态 `WAIT_TARGET`。
- 最终位置：`(-35.6368, -0.0604)`，平面误差约 `0.1496 m`，满足原控制器
  `finish_dist=0.15 m` 的边界。
- 未触发 MuJoCo 倾覆、过姿态或非有限状态故障。

## 观察项

高负载无界面运行中仍偶发很短的 `ODOM_STALE -> NAVIGATION` 状态切换，这是 ROS
回调调度抖动；本次没有中断规划，也没有导致 backend fault。官方 SDK 的最终
`0.45/0.20/0.65` 速度包线仍存在，属于下一阶段 M20 物理运动适配范围。
手动 Ctrl-C 时部分第三方/rclpy 节点打印 `exit code -2` 或 KeyboardInterrupt，均为
主动测试清理，不是运行故障。
