# SCAN-Planner 首场景移植验收

日期：2026-07-27

## 构建

`colcon build --symlink-install --packages-up-to m20_warehouse_inspection`
成功，18 个依赖包完成构建。

## 首层导航

使用统一入口、关闭 RViz 并启用图内 quick acceptance：

```bash
ros2 launch m20_warehouse_inspection inspection_mission_rviz.launch.py \
  use_rviz:=false run_acceptance:=true acceptance_mode:=quick
```

结果：

- 原 `pcl_render_node` 加载 F1 点云 135240 点；
- 地图范围 `(-45,-20,0)` 到 `(-5,20,2)`；
- SCAN 从 `(-42,0,0.59)` 规划到 `(-40,0,0.59)`；
- 生成初始 B 样条并执行一次原生滚动重规划；
- 返回 `PHASE4_QUICK_ACCEPTANCE_PASS`。

## 楼层切换与点云重建

执行一次 F1 到 F2 的事务回归：

```bash
ros2 launch m20_warehouse_inspection phase5_regression.launch.py \
  mode:=switch_stress switch_iterations:=1 use_rviz:=false
```

结果：

- F1 generation 1 正常进入安全保持；
- 地图提交为 F2 generation 2；
- 原渲染器打印 `Reloading global point cloud`；
- F2 重建后点数 137213，范围 `(5,-20,0)` 到 `(45,20,2)`；
- SCAN 在新位姿执行第二次 reset；
- 返回 `PHASE5_SWITCH_STRESS_PASS`，无保持期间非零速度；
- `pcl_render_node` 在回归结束后正常退出，无段错误。

## 自动化测试

执行 `m20_warehouse_inspection` 包测试并汇总当前工作区已有测试结果：

- 包内 13 个 CTest 项全部通过；
- 工作区汇总为 `226 tests, 0 errors, 0 failures, 0 skipped`；
- RViz 回归项以移植后的原版显示层、话题和样式为基准，不再验收旧的
  phase-2 定制界面。

## 结论

原版点云生成、原版 SCAN 规划/控制话题、M20 执行以及 F1→F2 PCD 重载链
均已联通。GUI 视觉验收仍应在用户桌面会话中运行统一入口观察。
