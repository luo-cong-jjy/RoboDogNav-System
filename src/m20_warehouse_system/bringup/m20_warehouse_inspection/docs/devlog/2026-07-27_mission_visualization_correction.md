# 0.8.0 任务终点、长路线与 RViz 配色校正开发记录

日期：2026-07-27  
范围：默认五步任务、自动任务规划 profile、双区域点云显示、围栏碰撞语义

## 1. 现场现象

完整任务进入原第 5 步后长时间不动；配置检查发现第 5 步仍指向
`F2_B=(35,10,0)`。同时完整任务入口在 0.7.0 模型/算法校正后默认使用纯原生 SCAN
目标链路，原生链路适合算法诊断，但对随机障碍中的长距离仓库任务缺少静态全局拓扑
引导。

RViz 中同一区域会随相机移动在黄色与深粉色之间变化。原因不是 PCD 被修改，而是：

1. `/m20/visualization/all_floors_cloud` 使用 `Intensity`，F1/F2 的 intensity 分别为
   1 和 2；
2. `/m20/map/active_global_cloud` 使用黄色 `FlatColor`；
3. active floor 同时存在于上述两个完全共面的显示中，移动相机时发生深度竞争。

## 2. 终点决策

两块区域范围为 F1 `x=[-45,-5]`、F2 `x=[5,45]`，中间 10 m 是隔离带。
`(0,0)` 不属于任何 active PCD 或 occupancy，不能成为可执行导航终点。

默认任务继续保持五步：

```text
F1_A -> F1_B -> E1:F1->F2 -> F2_A -> F2_TERMINAL
```

第 5 步使用新增的 typed `terminal` step，解析到 F2 电梯大厅
`(8,0,pi)`。该点位于 F2 内侧、与切图后的 release pose 一致。`F2_B` 继续作为可选
巡检点保留，但不进入默认演示任务。

## 3. 规划 profile

- `inspection_mission_rviz.launch.py` 默认 `use_grid_route=true`，使用 active occupancy
  A* 生成长路线和约 0.9 m 的顺序 SCAN 子目标。
- `f1_scan_rviz.launch.py` 与 `multifloor_scan_rviz.launch.py` 继续默认
  `use_grid_route=false`，保留原生 SCAN 目标、重规划和 B 样条链路用于人工/算法诊断。
- 完整任务仍可显式设置 `use_grid_route=false` 做对照实验。

该分层不复制或替换 SCAN 核心：A* 只承担静态全局引导，局部轨迹、重规划和控制仍由
`m20_scan_planner` 执行。

## 4. 显示数据面修正

地图服务器新增 transient-local 话题：

```text
/m20/visualization/inactive_floors_cloud
```

服务器启动时为每个 active floor 预计算“其余楼层”点云，切图提交时与 active cloud
同步切换。默认 RViz 改为只组合：

- active floor：黄色 `/m20/map/active_global_cloud`；
- inactive floor：蓝灰色 `/m20/visualization/inactive_floors_cloud`。

每个地图点只绘制一次，取消 `Intensity` 颜色和共面重叠。完整
`all_floors_cloud` 仍保留给分析工具，但默认 RViz 不叠加它。

SCAN 原始占据（黄色）和膨胀占据（半透明红色）仍作为两个调试层保留。红色膨胀层会
跟随局部滑动窗口，覆盖黄色点时局部呈粉红色，这是安全距离语义；与此前整张 active
PCD 因共面重复绘制而随相机变化的现象不同。

## 5. 围栏语义复核

RViz 的 `floor_boundaries` Marker 只做 active/inactive 状态高亮，不直接参加碰撞。
真实围栏由地图资产提供两份一致表达：

- PCD：`_wall_surface_points()` 在四周生成 2 m 高墙面，进入 active local sensing、
  SCAN occupancy 和 inflated occupancy；
- PGM：`_build_occupancy()` 把首末行、首末列全部置为 100，占据边进入栅格 A* 和
  `collision_guard`。

独立碰撞保护按 `0.38 m` M20 footprint 加 `0.12 m` 安全余量膨胀 active occupancy。
非活动楼层不会进入当前规划或保护链路。电梯切换在围栏内部的 cabin/release 点完成
停车、切图和仿真位姿迁移，不需要穿越围栏。

## 6. 实现清单

- `m20_inspection_core`
  - mission resolver 支持 `terminal`；
  - mission executor 增加无 dwell 的 terminal 执行分支；
  - 完成态 Marker/状态索引钳制为 `5/5`。
- `m20_warehouse_inspection`
  - 配置 schema 校验 terminal 的 floor、location 和 name；
  - 默认第 5 步改为 `F2_TERMINAL`；
  - 自动任务入口默认启用 grid route；
  - map server 发布 inactive-only 点云；
  - 两套 RViz 配置使用稳定 FlatColor；
  - full acceptance 最终点改验 `(8,0)`；
  - 增加终点、显示和围栏回归测试；
  - 项目 release 更新为 0.8.0。

## 7. 验证结果

```text
定向 pytest                         20/20 PASS
改动包 colcon build                 2/2 PASS
改动包 CTest                        22/22 PASS
当前工作区 test-result              215 tests, 0 failures
配置和地图资产校验                  PASS
无 GUI 完整五步运行                 PASS
pause/resume + MISSION_HOLD         PASS
F1 -> F2 generation                 1 -> 2 PASS
第 5 步                             F2_TERMINAL PASS
第 5 步路线                         16.23 m / 20 SCAN subgoals
第 5 步运行时间                     约 49.05 s
最终终点误差                        0.123 m（阈值 0.25 m）
```

完整任务从 STARTING 到验收 PASS 约 254.66 s。第 5 步持续发布子目标、轨迹和安全速度，
未出现无进展停滞。

本轮验收为 headless；RViz GUI 的人工截图复核留给用户启动默认入口后观察。显示配置、
互斥话题和 map server 发布链路已通过静态测试及完整节点图启动。
