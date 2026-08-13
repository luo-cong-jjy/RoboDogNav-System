# 0.9.0 SCAN-Planner 一致性边界开发记录

日期：2026-07-27  
对照基线：当前工作空间 `src/third_party/SCAN-Planner`  
范围：规划源码、参数、控制器、话题、RViz 显示、完整仓库任务

## 1. 严格结论

“整套双区域巡检系统与原项目完全一致”不是准确说法。准确边界如下：

| 层级 | 一致性结论 | 说明 |
| --- | --- | --- |
| SCAN 规划数学主链 | 一致 | `planner_manager.cpp` 去除注释/排版后与本地基线相同 |
| GridMap、Dynamic A*、rebound、B 样条优化、可视化发布器 | 直接复用 | `plan_env`、`path_searching`、`bspline_opt`、`traj_utils` 直接从第三方源码构建 |
| 原生规划参数 | 一致，除明确的 M20 覆盖 | 速度、加速度、规划视距、控制点距离、优化权重及 `dist0` 对齐 |
| 原生闭环控制参数 | 一致 | 在独立安全/运动后端限速前与 `controllers.yaml` 对齐 |
| SCAN RViz 核心图层 | 话题语义和显示属性对齐 | 使用 M20 命名空间；额外保留仓库、任务、M20 和安全图层 |
| SCAN FSM | 必要适配 | 增加楼层 reset、终点退出；Go2 执行冻结话题改为 M20 |
| 自动五步任务 | 不完全一致 | 默认增加外部二维 A* 长距离引导，局部轨迹仍全部由 SCAN 完成 |
| 地图和感知 | 项目适配 | 当前使用 generation-aware PCD 局部感知，不是原项目 OpenGL/Gazebo 雷达 |

因此项目对外应使用以下表述：

> 原生对照 profile 的 SCAN 核心、参数和 SCAN 可视化与当前工作空间第三方基线对齐；
> 完整仓库任务在 SCAN 外部增加楼层、任务、安全和静态全局路线适配。

## 2. 源码所有权和唯一实现

运行图只启动：

```text
m20_scan_planner/scan_planner_node
m20_scan_planner/closed_loop_controller
```

`m20_scan_navigation` 不再编译第二份 SCAN C++ 规划器或控制器，只提供：

- typed 导航 Action gateway；
- 可选静态栅格全局路线和顺序短子目标；
- 原生/栅格增强 profile 选择。

`m20_scan_planner/src/planner_manager.cpp` 与本地第三方
`plan_manage/src/planner_manager.cpp` 增加了自动回归：去除注释和空白后的源码必须完全
相同。这样注释整理不会造成误报，任何数学或控制流改动都会使测试失败。

底层算法库没有复制到集成包：

```text
plan_env        SCAN GridMap、占据/膨胀和滑动窗口
path_searching  SCAN Dynamic A*
bspline_opt     rebound 与 B 样条优化
traj_utils      轨迹消息和原项目 Marker 发布器
```

它们直接由 `src/third_party/SCAN-Planner` 中对应包构建并链接。

## 3. 明确保留的 FSM/控制器适配

以下差异不能删除，否则双区域切图、M20 命名和安全停车不成立：

1. `planning/go2_execution_frozen` 改名为 `planning/m20_execution_frozen`。
2. 增加 typed `/m20/navigation/reset`，切层时清空 GridMap、目标、全局/局部轨迹和
   active waypoints。
3. 增加 `fsm.target_reached_tolerance`，近终点发布停止轨迹并回到
   `WAIT_TARGET`，避免原来的近目标重规划循环。
4. 控制器保留 `forward_only` 兼容开关；原生 profile 为 `false`，不改变上游控制
   行为。
5. `cmd_vel` 重映射到 `/m20/navigation/cmd_vel_raw`，必须经过独立 supervisor 和
   collision guard 后才能进入运动后端。

这些属于接口、生命周期和安全适配，不改变 `planner_manager` 的初始轨迹、rebound
A*、优化和时间重分配主链。

## 4. 原生参数对齐

`f1_planner_native.yaml` 自动比较第三方 `planner.yaml` 的所有共同键。仅允许以下
M20 几何/显示覆盖：

```text
grid_map.body_height
grid_map.double_cylinder_offset
grid_map.obstacles_inflation_z_down
grid_map.obstacles_inflation_z_up
grid_map.sliding_map_frame_id
grid_map.vis_height
```

其余参数必须逐键相等，包括：

```text
resolution=0.05 m
planning_horizon=7.5 m
manager.max_vel=0.75 m/s
manager.max_acc=0.50 m/s²
control_points_distance=0.20 m
optimization.dist0=0.20 m
```

新增 `f1_controller_native.yaml`，其共同参数逐键对齐第三方
`controllers.yaml`。独立安全 supervisor 和 RViz 运动后端仍将最终可执行速度限制为
`0.45/0.20/0.65`，该限制不伪装成 SCAN 算法参数。

## 5. RViz 可视化对齐

默认 RViz 的以下 SCAN 图层恢复本地第三方 `default.rviz` 的 Class、Alpha、颜色变换、
Style 和点/线尺寸：

| 集成显示名 | M20 话题 | 原项目语义 |
| --- | --- | --- |
| SCAN Global Map | `/m20/map/active_global_cloud` | Global Map |
| SCAN Sensor Cloud | `/m20/sensing/sensor_cloud` | 实时 Sensor Cloud |
| SCAN Occupancy | `/m20/navigation/grid_map/occupancy` | GridMap 占据 |
| SCAN Inflated Occupancy | `/m20/navigation/grid_map/occupancy_inflate` | 膨胀占据 |
| SCAN Robot Path | `/m20/sim/path` | Robot Path |
| SCAN Sliding Map Bounds | `/m20/navigation/grid_map/sliding_map_bbox` | 滑窗边界 |
| SCAN Goal | `/m20/navigation/goal_point` | 目标点 |
| SCAN Global Path | `/m20/navigation/global_list` | 全局参考 |
| SCAN Initial Path | `/m20/navigation/init_list` | 优化初值 |
| SCAN Rebound AStar | `/m20/navigation/a_star_list` | rebound A* 段 |
| SCAN Optimized Path | `/m20/navigation/optimal_list` | 优化 B 样条 |

原项目默认 RViz 主要打开 `optimal_list`，但核心实际发布后四个轨迹 Marker。本系统将
它们全部加入 Displays，属于原项目调试输出的可见超集，没有用自定义轨迹替换 SCAN
轨迹。

`/m20/sensing/local_cloud` 是规划器真正消费的点云。它与当前
`sensor_cloud` 数据一致，另作为默认关闭的 `SCAN Planner Input Cloud
(Supplement)` 保留，便于检查感知发布器和规划器输入是否一致。

仓库集成额外显示官方 M20、inactive floor、楼层/任务 Marker、TF 和机体安全轮廓。
可选 `/m20/navigation/global_route` 明确命名为 `AStar Global Route`，默认关闭，
不会被误认为 SCAN 的 `global_list`。

## 6. 为什么完整任务默认保留全局引导

本轮对 `inspection_mission_rviz.launch.py use_grid_route:=false` 做了三次无 GUI
完整任务试验：

1. 旧的 0.50 m 独立保护半径；
2. 与 SCAN 横向机体圆柱对齐的 `0.25 + 0.05 = 0.30 m`；
3. 0.30 m 半径，并把碰撞预测速度钳制为后端实际可执行的
   `0.45/0.20/0.65`。

三次均确认 gateway 为 `mode=native_scan`，且没有启动
`m20_grid_route_planner`。SCAN 持续成功生成朝 F1_A 的局部轨迹，但全局参考仍从障碍
同一侧接近；独立静态碰撞保护分别在约 `(-37.9,-5.85)`、
`(-37.7,-5.99)` 和 `(-37.5,-6.23)` 安全截停。任务没有通过第 1 步。

这不是关闭安全层的理由。纯原生 SCAN 在本场景是局部规划算法对照 profile，不应被
包装成随机障碍仓库的全局连通性保证。

最终策略：

- `f1_scan_rviz.launch.py`、`multifloor_scan_rviz.launch.py` 默认
  `use_grid_route=false`，用于人工目标和 SCAN 算法核对；
- `inspection_mission_rviz.launch.py` 默认 `use_grid_route=true`，用于完整仓库任务；
- 外部 A* 只确定静态拓扑并下发约 0.9 m 短目标，SCAN 仍生成、重规划和控制每一段
  B 样条；
- `use_grid_route=false` 始终保留，可直接复现实验。

## 7. 独立安全保护修正

碰撞保护的预测速度现在先钳制到 supervisor/运动后端最大速度，再进行 1 s 前视。
此前用 SCAN 原始 `0.75 m/s` 预测、但后端最多执行 `0.45 m/s`，会让保护层检查机器人
不可能在该周期到达的位置。

两个 profile 保持隔离：

```text
native SCAN       0.25 m footprint + 0.05 m margin
grid-route task   0.38 m footprint + 0.12 m margin
```

这项修正只让预测与实际命令一致，不取消 occupied-grid 碰撞检查。

## 8. 使用方式

日常操作只使用一个系统入口。步骤一启动完整系统：

```bash
ros2 launch m20_warehouse_inspection inspection_mission_rviz.launch.py
```

此时可以直接在 RViz 使用 `2D Goal Pose` 做 SCAN 自由导航。步骤二仅在需要自动巡检时
从另一个终端触发已经运行的 mission executor：

```bash
ros2 run m20_warehouse_inspection m20_start_inspection
```

`f1_scan_rviz.launch.py`、`multifloor_scan_rviz.launch.py` 和
`use_grid_route` 参数只属于开发回归接口，不再作为操作者的启动选择。
