# 2026-07-31 SCAN 原项目基线恢复

## 决策

现场现象更符合“全向 SCAN 速度无法被轮腿 M20 准确兑现”，而不是 SCAN 原规划
算法本身失效。为隔离变量，本轮撤回所有会改变 SCAN 路径和闭环行为的 M20 专用
试验，只在下游运动适配层继续定位。

## 当前有效参数

完整 RViz、自动巡检和 MuJoCo 主入口均直接加载：

```text
scan_vendor_planner.yaml
scan_vendor_controller.yaml
clearance_vendor.yaml
```

与固定第三方 `SCAN-Planner/plan_manage/config` 相同的核心值为：

```yaml
manager.max_vel: 0.75
optimization.max_vel: 0.75
grid_map.double_cylinder_radius: 0.25
grid_map.double_cylinder_offset: 0.18
optimization.dist0: 0.20
heading_error_threshold: 0.80
max_vx: 0.75
max_vy: 0.35
max_vyaw: 1.0
```

## 已撤回内容

- `0.40 m/s` SCAN planner/controller 覆盖；
- `0.30/0.15 m` 硬半径/优化距离覆盖；
- 配置空间净空查询与 rebound A* 净空边代价；
- B-spline 全局净空梯度；
- 航向进入/退出迟滞、转向前平移缩放和最小保持时间；
- SCAN 控制器的航向误差/对正状态诊断话题。

## 保留的必要集成

- `body_pose`、点云和原始速度话题映射；
- `0.20 m` 任务到点退出，避免 Action 与原 FSM 终点状态不一致；
- typed 楼层 reset；
- `/m20/control/execution_hold`：碰撞保持、任务暂停和切层事务期间冻结轨迹时钟；
- 下游 `m20_navigation_adapter`、独立双圆碰撞保护、安全 supervisor、官方 SDK
  和 MuJoCo 后端。

控制边界为：

```text
vendor SCAN
  -> cmd_vel_raw
  -> M20 motion adapter
  -> cmd_vel_candidate
  -> collision/safety gate
  -> cmd_vel_safe
  -> official SDK / MuJoCo
```

因此后续转向半径、横移转换、轮腿协调和跟踪误差的修改不应再进入 SCAN 参数或
优化器，而应在 `m20_navigation_adapter` 及其反馈闭环中完成。

## 验证口径

1. vendor 配置的所有上游键逐项相等；
2. 主启动不存在 planner 速度、conservative clearance 或 physical controller
   覆盖；
3. `planner_manager.cpp` 与固定第三方副本进行去注释/布局后的源码一致性测试；
4. 控制器不存在航向迟滞/平移缩放符号，只保留 external hold；
5. 相关包重新构建并运行导航/系统契约回归。

