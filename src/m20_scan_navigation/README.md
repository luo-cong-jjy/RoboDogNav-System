# M20 SCAN Navigation

本包是仓库系统与 SCAN-Planner 之间的 Python 组合/协议层，不再复制或编译 SCAN
算法。唯一的 SCAN 重规划 FSM、GridMap、路径搜索、B 样条优化和闭环控制实现位于
`m20_scan_planner`。

本包负责：

- 启动 SCAN 原版 CPU local sensing、`m20_scan_planner` 和安全速度重映射。
- 提供携带 `floor_id + map_generation` 的 `/m20/navigation/navigate` Action gateway。
- 提供可选的 active-occupancy 膨胀 A* 路线与顺序短子目标适配器。
- 在取消、超时、任务暂停和切层时协调路线及 SCAN reset。

独立 `f1_scan.launch.py` 默认 `use_grid_route:=false`，用于复现原版话题链：

```text
/move_base_simple/goal
  -> m20_scan_planner
  -> closed_loop_controller
  -> /m20/navigation/cmd_vel_raw
```

默认模式读取 `config/scan_vendor_planner.yaml` 和
`config/scan_vendor_controller.yaml`。GridMap 分辨率、7.5 m 规划视野、机器人
碰撞几何、0.75 m/s 规划上限、优化权重和闭环参数逐项对应固定第三方
`plan_manage/config`。仅额外提供 0.20 m 到点退出与 typed 楼层 reset。

直接启动 `f1_scan.launch.py` 和完整 M20 仓库系统现在都保留上述原版参数：
`manager.max_vel=0.75 m/s`、`optimization.max_vel=0.75 m/s`、
`grid_map.double_cylinder_radius=0.25 m` 和 `optimization.dist0=0.20 m`。
主启动不再叠加 M20 专用规划速度、硬膨胀或朝向控制参数，也没有额外的 A*/
B-spline 净空代价。

`m20_scan_planner` 的闭环速度和朝向门限同样回到原项目算法。控制器保留
`/m20/control/execution_hold`，在任务暂停、碰撞保持或原子切层时冻结 B-spline
执行时钟。另有一个默认关闭的平台能力扩展：完整 M20 系统可注入双向跟踪参数，
让位于机身后方的同一条 B-spline 以负 `vx` 执行，并在
`/planning/tracking_direction` 发布方向。独立 `f1_scan.launch.py` 默认关闭该扩展，
因此仍保持原项目执行语义。其余 M20 非完整约束、滚动优先转换、SDK 限幅和实体
碰撞否决均位于下游：

```text
SCAN cmd_vel_raw
  -> m20_navigation_adapter
  -> cmd_vel_candidate
  -> collision_guard / safety supervisor
  -> cmd_vel_safe
  -> official M20 SDK + MuJoCo
```

这种边界保证后续运动适配试验不会再改变 SCAN 的规划路径、重规划条件或原版
可视化。`clearance_conservative.yaml` 仅作为旧测试入口兼容文件保留，其 SCAN
参数也已恢复为原版 `0.25/0.20 m`。

局部点云读取 `config/scan_vendor_local_sensing.yaml`，使用第三方
`local_sensing_node/pcl_render_node` 发布 `/quad_0/cloud`。集成只开启默认关闭的
地图重载开关，以便切层后重建原 renderer 索引。

完整 M20 巡检默认启用以下仓库静态栅格长路线；独立 SCAN 对照可显式启用：

```text
use_grid_route:=true

/m20/navigation/goal_pose
  -> inflated-grid A*
  -> /m20/navigation/global_route
  -> sequential /m20/navigation/scan_goal
  -> m20_scan_planner
```

兼容模式继续使用同一套 vendor SCAN 参数，并在其外层使用 0.60 m 障碍硬膨胀。
硬膨胀外再计算 0.20 m、权重 4.0 的二次软净空代价：开阔区域优先走远离障碍的
路线，只有该软带是唯一连通通道时才允许进入，因此不会把仍可通行的窄道改成占据区。
路径简化和 SCAN 子目标压缩使用同一净空约束，不能在 A* 之后重新抄近路。子目标间距
为 0.90 m，普通/最终目标接受半径分别为 0.30/0.20 m；最终值与 SCAN 到点阈值一致，
避免 SCAN 已停车而适配器误报 `SUBGOAL_STALLED`。相邻段转角不超过 0.70 rad、
`0.54 m` 最小中心线转弯圆角能够放入两段、且提前交接到下一目标的整条捷径均位于
`0.60 m` 硬膨胀之外并再留 `0.10 m` 时，适配器会在距逻辑子目标 0.55 m 处发布下一
目标，让 SCAN 保留当前速度生成连续 B-spline。`0.893 m` 外扫半径只用于转弯空间
诊断，不被重复当作圆形膨胀。急转角和净空不足处仍使用实测里程计 stop-to-stop；若
6 s 内无法满足 `0.04 m/s`、`0.08 rad/s` 并稳定 0.30 s，则 fail closed 为
`SUBGOAL_STOP_TIMEOUT`。等待期间 `/m20/control/route_segment_hold=true` 会经 safety
supervisor 冻结旧 SCAN 轨迹；下一子目标在保持状态下先发布，0.20 s 后才释放。
边界处不支持原地转向时，可在同一膨胀自由区内先执行 0.90～1.20 m 直线倒车。它是
完整 M20 的上层引导，不修改 SCAN 算法。

一次实时故障证明“原生 SCAN 先走、保护耗尽后再接管”会接管过晚：M20 已驶入一条
可以直行但没有滚动转弯空间的 1.11～1.28 m 通道。相同起点和目标从运动前就使用净空
路线后动态通过。因此完整 `inspection_mission_rviz/mujoco.launch.py` 现在默认
`use_grid_route:=true`；SCAN 仍负责每一段的局部 rebound、B-spline、控制与可视化。
`collision_grid_route_enabled` 继续提供有界重试。位于当前机身后方超过 2.10 rad、且
距离超过 0.35 m 的目标使用双向控制，以直线倒车优先。独立 `f1_scan.launch.py`
仍默认关闭 M20 路线，`use_grid_route:=false` 也始终保留用于受控对照。

导航默认采用 `position_only` 终点语义。SCAN 只读取目标 XY，RViz/Action 中的目标
四元数不会成为严格终点朝向；控制器只跟踪 B-spline 当前切向。若巡检业务必须在终点
满足指定 yaw，需要另行加入“可转身区域搜索 -> 中间位置目标 -> 终点位姿对齐”的
turn-pocket 规划器，不能仅靠放大 yaw 容差或在狭窄通道内请求原地转向。

`/m20/navigation/reset` 由 `m20_scan_planner` 提供，负责清空 GridMap、目标、局部/
全局轨迹并让 FSM 回到 `WAIT_TARGET`。`/m20/navigation/route_reset` 只清空可选 A*
路线。gateway 在原生模式下不会等待或调用路线适配器。

本包和 `m20_scan_planner` 都不直接发布后端 `/cmd_vel`，也不依赖 Gazebo、MuJoCo
或 M20 SDK；运动命令必须继续经过系统安全层。
