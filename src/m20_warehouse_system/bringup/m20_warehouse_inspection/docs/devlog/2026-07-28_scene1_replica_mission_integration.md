# 场景 1 复制与巡检切图融合记录

日期：2026-07-28

## 目标

保留已经对齐原版 SCAN-Planner 的首场景，将该场景严格复制为第二块平面区域。F2 不再
独立随机生成障碍物；自动巡检在同一个 SCAN 核心上完成 F1 导航、切换活动 PCD、重置
规划状态和 F2 导航。

## 地图资产合同

系统配置为 F2 增加：

```yaml
replica_of: F1
source_seed: 127
simulation_offset: [25.0, 0.0, 0.0]
```

F1 的偏移为 `[-25,0,0]`，因此两个场景中心间距为 50 m。生成器使用 F1 的 seed、保护
走廊和障碍列表生成 F2，并保留相同 intensity。F1/F2 的 PCD 和 PGM 必须具有相同
SHA-256；验证器会拒绝不完整的副本。

F2 巡检点使用与 F1 相同的 floor-local 坐标：

| 逻辑点 | F1 展示坐标 | F2 展示坐标 | floor-local |
|---|---:|---:|---:|
| A | `(-35,-10)` | `(15,-10)` | `(-10,-10)` |
| B | `(-20,10)` | `(30,10)` | `(5,10)` |

## RViz 语义

- `Global Map` 仍是原版 SCAN active map，保持原版规划显示和话题。
- `Copied Scene (Inactive Floor)` 显示另一块平移后的副本，采用固定蓝灰色。
- 地图服务器只向 active map 发布当前楼层；未激活副本不进入 SCAN/local sensing。
- 切换后两层交换 active/inactive 身份，因此不会发生同一点云双层重绘。

## 巡检与切图事务

默认任务保持五个步骤：

```text
F1_A -> F1_B -> E1:F1->F2 -> F2_A -> F2_TERMINAL
```

电梯步骤内部执行：

```text
导航 F1 lobby/cabin
-> 断言 floor_switch_hold 并确认零速
-> reset F1 SCAN
-> 提交 F2 PCD/occupancy，generation + 1
-> 传送到 F2 内侧电梯点 (8,0,0)
-> reset F2 SCAN
-> 等待至少 3 帧新 generation 局部点云
-> 释放安全保持并继续 F2 任务
```

最后一步返回 F2 内侧电梯大厅 `(8,0,0)`。隔离带中心 `(0,0)` 不属于任何 PCD，仍不
作为导航终点。

## 模块边界

- `m20_warehouse_inspection`：副本配置、资产生成、地图发布和任务入口。
- `m20_inspection_core`：巡检编排和事务式楼层切换。
- `m20_multifloor_map`：监听原版 `/quad_0/cloud` 并确认新 generation 感知新鲜度。
- `m20_scan_planner`：继续使用原版目标、局部地图、B 样条和控制链，仅接受楼层 reset。

副本逻辑没有进入 SCAN 规划算法，后续实物部署只需替换地图定位/电梯执行后端。
