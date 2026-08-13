# 2026-08-02 M20 边界恢复与膨胀区复验

## 问题边界

此前的 `The robot is inside an obstacle` 不能直接等同于 MuJoCo 机身碰到实体。
SCAN A* 判断的是离散占据/膨胀栅格；0.10 m 栅格在连续半径离散化时还会产生最多半个
栅格的保守外壳。另一个固定风险位于共享原点：F1 active occupancy 在 `x=0` 结束，
机器狗从略大于零的切图位姿向 F1 倒车时，后圆预测会先进入地图外区域。

完整任务还暴露出第三个独立问题。栅格后备路线到达中间子目标的接受半径后，旧 SCAN
轨迹仍可能带速收尾；若立刻发布下一条不同方向的线段，M20 会用滚动曲线连接两段，
机身外扫可能离开两条静态直线都安全的走廊。

本轮不修改 vendor SCAN 的 A*、rebound、B-spline、控制参数或 RViz 显示，只在执行
安全边界和后备路线换段处处理这三类情况。

## 实现

### 1. 区分保守膨胀外壳与实体机身占用

`m20_collision_guard` 同时维护两份栅格：

- 保守安全栅格用于正常候选命令预测；
- 不含额外安全余量的 hard-body 栅格只用于判定当前是否真的存在机身占用风险。

只有“保守栅格报告当前占用、hard-body 栅格明确自由、整条直线 hard-body 扫掠自由、
且终点重新进入保守自由区”四个条件同时成立时，才允许一次有界直线脱壳，并发布
`RASTER_SHELL_ESCAPE`。hard-body 重叠时仍为零速，不能借该分支穿过实体障碍。

### 2. 共享地图边缘向内恢复

当倒车候选只因 active occupancy 边缘产生 `OUT_OF_BOUNDS` 时，保护器可以选择经过
完整扫掠验证的零偏航直线前进，把前后双圆带回当前地图内部，并发布
`MAP_EDGE_INWARD_RECOVERY`。该恢复仍受既有的 6 s、0.75 m 总预算和进度看门狗约束；
不允许纯偏航，也不允许带偏航倒车。

安全 supervisor 的最终门同步接受保护器已验证的直线前进或滚动前进恢复，继续拒绝
纯偏航、横移和带偏航倒车，避免“保护器判安全、最终门又永远清零”的接口矛盾。

### 3. 后备路线段间显式保持

`m20_grid_route_planner` 新增 `/m20/control/route_segment_hold`。中间子目标进入接受半径
后立即断言该保持，安全 supervisor 输出零速并冻结旧 SCAN 执行时钟。只有里程计满足：

- 线速度不大于 `0.04 m/s`；
- 角速度不大于 `0.08 rad/s`；
- 连续稳定至少 `0.30 s`；

才在保持状态下发布下一 SCAN 子目标；再等待 `0.20 s` 让新轨迹回调到达后释放保持。
若 6 s 内未停稳，路线以 `SUBGOAL_STOP_TIMEOUT` fail closed。

### 4. 固定冷启动探针

新增 `boundary_recovery_mujoco.launch.py` 和 `m20_boundary_recovery_probe`，固定覆盖：

- `upper_boundary`：从 `(-7.2, 16.8, -3.11)` 导航到 `(-36, 16)`；
- `shared_origin_exit`：从 `(0.10, 0.12, -2.63)` 导航到 `(-37, 0)`。

探针记录导航 Action、机身/SCAN/保护层净空、MuJoCo 接触对与峰值力、保护器诊断、
后备路线状态、子目标发布前停稳时间和后端故障。MuJoCo 初始位姿覆盖参数默认为空；
正常主启动仍从系统 YAML 的 F1 起点开始，不增加运行时传送服务。

## 架构与默认值

- 正常路线仍是 `/move_base_simple/goal -> vendor SCAN -> cmd_vel_raw`；
- 主入口仍默认 `use_grid_route=false`，仅在有界碰撞恢复无法继续时启用后备路线；
- SCAN 仍使用原版 `0.25 m` 硬双圆半径和 `0.20 m` 优化距离；
- native-SCAN 下游保护仍使用 `0.25 + 0.05 = 0.30 m` 连续包络；
- 后备栅格路线仍使用独立的 `0.60 m` 膨胀，不用缩小膨胀换取通过；
- 官方 M20 MJCF、ONNX 策略、16 执行器 SDK 接口均未修改。

动态证据和放行边界见
`docs/test_reports/2026-08-02_boundary_recovery_cold_starts.md`。

