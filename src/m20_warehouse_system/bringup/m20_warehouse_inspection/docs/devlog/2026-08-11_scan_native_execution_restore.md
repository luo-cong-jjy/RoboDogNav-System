# 2026-08-11 原生 SCAN 执行链恢复记录

## 结论

附件故障日志中，SCAN 本体持续处于 `EXEC_TRAJ`，重规划多次得到
`final_plan_success=1`；停车来自下游的 `PREDICTED_FOOTPRINT`、
`RECOVERY_BUDGET_EXHAUSTED`、`COLLISION_STOP` 和 external hold 反复切换。
因此本轮没有继续调整膨胀半径或 B-spline 代价，而是恢复原项目的职责边界。

## 默认链路

`execution_profile:=scan_native` 现在是 RViz 和 MuJoCo 主入口默认值：

```text
RViz/任务目标
  -> 原生 SCAN GridMap + rebound A* + B-spline
  -> 原生 closed_loop_controller 全向 Twist
  -> 多层/急停透明门控
  -> RViz 运动学后端，或 MuJoCo + 官方 M20 SDK
```

默认模式不启动：

- `m20_navigation_adapter`；
- `m20_collision_guard`；
- 预测足迹、恢复预算和恢复指令；
- 外部 safety hold 对 SCAN 轨迹时钟的冻结；
- M20 双向跟踪扩展。

多层 PCD、目标 Action、任务执行、切层停车、急停和后端就绪仍保留。这些属于系统
集成边界，不改变 SCAN 的搜索和轨迹生成。

## SCAN 源码恢复

- 删除 `planner_manager.cpp` 中 M20 短路线 1.10 倍循环时间拉伸；该文件当前与
  `src/third_party/SCAN-Planner` 逻辑一致。
- 删除集成配置中的 5 Hz 可视化降频覆盖，planner 参数键和值与上游完全一致，
  使用原生 20 Hz 默认值。
- 删除 collision hold 后强制重规划导数清零。
- 删除集成层增加的近终点提前退出。
- 保留 `/m20/navigation/reset` 服务，用于切换 active PCD 时原子清空旧楼层
  GridMap、目标和轨迹。
- 控制器中的 M20 扩展代码暂留以便 A/B，但默认参数为
  `bidirectional_tracking_enabled=false`、
  `require_external_execution_hold=false`；未启用时数值控制路径与上游一致。

## 可选旧链

历史实验链保留为：

```bash
ros2 launch m20_warehouse_inspection inspection_mission_mujoco.launch.py \
  execution_profile:=m20_safe
```

该模式才启动滚动适配、足迹 guard、恢复和轨迹 hold。保留它是为了后续定量比较，
不是当前默认方案。真机入口仍固定到 `m20_safe`，直到实体空场和障碍验收完成。

## 已知 M20 边界

MuJoCo/真机不能等价于原项目的理想 Go2 全向质点。当前 `scan_native` 会把原始
Twist 保留到官方接口前，但官方 SDK 仍按已验证能力限制为
`0.45 m/s / 0.20 m/s / 0.65 rad/s`。轮足策略对横移、曲线和急转的真实响应属于
下一阶段运动层适配，不再通过修改 SCAN 规划参数掩盖。
