# 相邻双场景与共享原点切图开发记录

日期：2026-07-28  
版本：1.1.0  
范围：RViz 平面仿真、地图资产、楼层事务、任务回归

## 1. 需求调整

本次把早期“左右分离区域 + 仿真位姿传送”调整为更直观的平面近似：

- F1 与 F2 同处 `z=0`，面积均为 40 m × 40 m，并在 `x=0` 相邻。
- F1 右围栏与 F2 左围栏在共享原点附近保留通道。
- 启动时同时显示两块场景。
- 自动巡检必须先让 M20 自主导航到 `(0,0)`，停车后才切换 active PCD/occupancy。
- 当前默认流程严禁瞬移；切图前后 odom 连续。
- 人工目标和自动任务继续使用移植后的原版 SCAN 点云、重规划、B 样条、闭环控制和
  RViz 调试显示。

## 2. 配置单一事实源

`config/flat_multifloor_system.yaml` 调整为：

```text
layout_mode: flat_connected_regions
F1 simulation_offset: [-20, 0, 0]  -> x=[-40, 0]
F2 simulation_offset: [ 20, 0, 0]  -> x=[  0,40]
inter_region_gap: 0
teleport_on_floor_switch: false
source_trigger_pose: [0,0,0]
target_release_pose: [0,0,0]
```

两侧 `transition_gateway` 宽度均为 4 m，中心均为 `y=0`。配置校验器同时约束：

- connected profile 的区域只能接边、不能重叠或留间隔；
- trigger 与 release 必须是同一世界坐标；
- 触发点必须位于两侧门洞中心；
- connected profile 禁止直接跨区推断楼层，且禁止启用传送；
- 平面门洞只按位置触发，不强制机器狗朝向。

## 3. 地图资产

地图生成器在 PCD 墙面和 PGM 占用边上同时移除门洞，不是只隐藏 RViz Marker。
F2 继续复用 F1 的 180 个内部障碍；由于门洞位于相反边界，完整 PCD/PGM 哈希有意
不同，内部点云、内部栅格和障碍 JSON 必须完全相同。

原版 SCAN 默认直接接收最终目标，不使用定制 Manhattan/A* 子目标链。为提高自动任务
的可重复性，生成器从 mission sequence 推导每个直达航段，并用 1 m clearance 的
重叠采样区域约束确定性障碍生成。该处理不替换 SCAN 规划，只避免配置生成器把固定
障碍放入自动任务的直达走廊。

最终资产：

```text
F1: 144586 points, 180 obstacles
F2: 144586 points, 180 obstacles
F1 PCD: 1763a3ceefba9e7a480dc53077f822532e51aed841b707d4f7064b4f89b83054
F2 PCD: 34ea9a1158dbc13a37a6150892a51d48f05da92aa71ccdfd0e980411e3fe7f8b
```

## 4. 启动显示

地图服务器在首次锁存发布中同时发送：

- `/m20/map/active_global_cloud`
- `/m20/visualization/inactive_floors_cloud`
- `/m20/visualization/all_floors_cloud`

默认 RViz 视角以原点为中心、距离 92 m，可在启动时覆盖 `x=[-40,40]` 的两块场景。
active 与 inactive 图层互斥，避免同一共面点云重复绘制产生黄/粉色深度竞争。规划器
只消费 active cloud；inactive 与 all-floors 仅用于显示。

边界 Marker 改为逐线段绘制，并在共享门洞处断开。Marker 只表达状态，真实碰撞仍由
PCD 墙面和 occupancy 边界负责。

## 5. 无瞬移楼层事务

`m20_floor_switch_manager` 根据配置选择事务策略：

- `teleport_on_floor_switch=false` 时不创建 `SetSimulationPose` 客户端；
- M20 到触发点并连续停车后，先保持零速、reset 旧导航，再提交 F2 地图；
- 地图提交后执行 `VERIFYING_SHARED_GATEWAY`，重新核对当前 odom 仍在 release 容差内；
- reset 新导航并等待新 generation 的本地点云后才释放 hold。

旧分离区域测试 profile 仍可显式启用传送，便于保留历史回归；它与当前默认 connected
profile 隔离。

切图后 odom 可能因闭环停车落在共享边界任一侧几厘米。碰撞保护增加可配置的
`map_edge_tolerance=0.35 m`，只把该窄带映射到边界栅格。没有门洞的边界单元仍为
occupied，因此容差不会打开其他围栏；超出 0.35 m 仍 fail-closed。

## 6. 任务与验收

默认五步任务为：

```text
F1_A -> F1_B -> E1:F1->F2 -> F2_A -> F2_TERMINAL
```

第 3 步包含“从 F1_B 导航到原点、停车、原地切图”；第 5 步从 F2_A 返回原点。
验收节点记录切换前最后一个 F1 odom 和切换后第一个 F2 odom，独立计算
`switch_jump`，不能只根据地图 generation 判断成功。

对应结果见
`docs/test_reports/2026-07-28_connected_origin_map_switch.md`。

## 7. 主要修改文件

```text
config/flat_multifloor_system.yaml
m20_warehouse_inspection/configuration.py
m20_warehouse_inspection/map_assets.py
m20_warehouse_inspection/map_server_node.py
m20_warehouse_inspection/phase4_acceptance_node.py
launch/multifloor_scan_rviz.launch.py
maps/floor_1/*
maps/floor_2/*

m20_inspection_core/floor_switch_manager_node.py
m20_inspection_core/collision_policy.py
m20_inspection_core/collision_guard_node.py

m20_scan_planner/launch/default.rviz
m20_warehouse_sim/rviz_kinematic_backend_node.py
```

