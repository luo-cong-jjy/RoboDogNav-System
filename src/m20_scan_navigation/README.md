# M20 SCAN Navigation

本包是仓库系统与 SCAN-Planner 之间的 Python 组合/协议层，不再复制或编译 SCAN
算法。唯一的 SCAN 重规划 FSM、GridMap、路径搜索、B 样条优化和闭环控制实现位于
`m20_scan_planner`。

本包负责：

- 启动 SCAN 原版 CPU local sensing、`m20_scan_planner` 和安全速度重映射。
- 提供携带 `floor_id + map_generation` 的 `/m20/navigation/navigate` Action gateway。
- 提供可选的 active-occupancy 膨胀 A* 路线与顺序短子目标适配器。
- 在取消、超时、任务暂停和切层时协调路线及 SCAN reset。

默认 `use_grid_route:=false`，手动和任务目标均进入原版话题：

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

局部点云读取 `config/scan_vendor_local_sensing.yaml`，使用第三方
`local_sensing_node/pcl_render_node` 发布 `/quad_0/cloud`。集成只开启默认关闭的
地图重载开关，以便切层后重建原 renderer 索引。

需要复用阶段 2～5 的仓库静态栅格长路线时，可显式启用兼容模式：

```text
use_grid_route:=true

/m20/navigation/goal_pose
  -> inflated-grid A*
  -> /m20/navigation/global_route
  -> sequential /m20/navigation/scan_goal
  -> m20_scan_planner
```

兼容模式继续使用同一套 vendor SCAN 参数，并在其外层使用 0.60 m 障碍
膨胀、0.90 m 子目标间距、0.30 m 普通子目标接受半径和
0.15 m 最终目标接受半径。它是仓库长距离任务的可选上层引导，不修改 SCAN 算法。

`/m20/navigation/reset` 由 `m20_scan_planner` 提供，负责清空 GridMap、目标、局部/
全局轨迹并让 FSM 回到 `WAIT_TARGET`。`/m20/navigation/route_reset` 只清空可选 A*
路线。gateway 在原生模式下不会等待或调用路线适配器。

本包和 `m20_scan_planner` 都不直接发布后端 `/cmd_vel`，也不依赖 Gazebo、MuJoCo
或 M20 SDK；运动命令必须继续经过系统安全层。
