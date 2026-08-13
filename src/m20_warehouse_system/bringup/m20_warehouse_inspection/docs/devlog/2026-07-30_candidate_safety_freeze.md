# 候选速度、安全预测与轨迹冻结一致性修复

日期：2026-07-30

## 问题

正式密集场景绕障时出现大量以下日志并停在中途：

```text
A-star failed; aborting optimization
First three control points are in obstacles
```

历史运行中，B 样条优化器内部 A* 失败 769 次、前三控制点入障碍 702 次，独立碰撞
保护进入 `COLLISION_STOP` 28 次。对应 A* 不是全局目标搜索，而是 rebound 优化器
修复局部碰撞段时使用的引导搜索。

根因是控制链语义不一致：

```text
SCAN raw vx/vy/wz
  -> collision guard 和 safety 按全向命令预测
  -> safety 之后才把 vy 转成 rolling yaw
  -> MuJoCo/SDK 执行另一条运动
```

停车期间 closed-loop controller 的 `exec_time` 和 SCAN 局部轨迹时间仍继续推进；
恢复规划又从旧 B 样条读取速度和加速度。实际机器人未执行的轨迹导数因此把新轨迹
起始控制点拉入膨胀障碍。

## 修复架构

当前自动速度链固定为：

```text
/m20/navigation/cmd_vel_raw
  -> m20_navigation_adapter
  -> /m20/navigation/cmd_vel_candidate
       -> m20_collision_guard
       -> m20_safety_supervisor
  -> /m20/control/cmd_vel_safe
       -> RViz backend
       -> m20_locomotion_manager（仅末端包络/后端门控）
  -> official M20 SDK
```

`m20_navigation_adapter` 复用既有 `RollingNavigationAdapter`，但把转换位置前移到安全
预测之前。自动 candidate、碰撞前视、安全门控和后端执行现在使用同一 `vx/vy/wz`
语义。人工速度仍从独立 manual 入口进入 safety supervisor，不丢失人工横移能力。

末端 `m20_locomotion_manager` 的 `rolling_navigation_enabled` 改为 `false`，防止
candidate 经过安全层后再次被转换。

## 轨迹时钟冻结

safety supervisor 新增 transient-local
`/m20/control/execution_hold`：

- `NAVIGATION` 发布 `false`；
- `MANUAL`、急停、碰撞停车、地图未就绪、切层保持、任务保持、里程计超时和命令
  超时均发布 `true`。

closed-loop controller 合并航向对齐冻结和外部保持：

- 保持期间不推进 `exec_time`；
- 继续发布“若解除安全保持将执行”的 candidate，避免 collision guard 因读到下游零
  速度而错误解除自身停车；
- 下游 safety supervisor 仍是唯一实际零速度门。

SCAN FSM 已有的 `planning/go2_execution_frozen` 回调继续平移局部轨迹
`start_time`。本次额外记录冻结沿，在保持触发后的恢复规划中：

- `start_pt = odom_pos`；
- `start_vel = 0`；
- `start_acc = 0`；
- 不再读取旧 B 样条导数。

## 碰撞几何与诊断

独立保护由单一中心圆改为与 SCAN 一致的定向双圆：

```text
front = body_center + 0.18 m * heading
rear  = body_center - 0.18 m * heading
radius = 0.25 m + 0.05 m independent margin
```

每个预测位姿检查前后两个圆心。新增
`/m20/control/collision_guard_diagnostic`，首次进入停车时记录：

- 当前 footprint 或未来预测；
- 首个触发时间；
- front/rear；
- 世界坐标和栅格；
- OCCUPIED/OUT_OF_BOUNDS；
- 被检查的 candidate 命令。

## 代码边界

修改只位于项目自有集成包：

- `m20_locomotion_control`：预安全导航适配节点；
- `m20_inspection_core`：execution hold、双圆保护和诊断；
- `m20_scan_planner`：控制器冻结与恢复起始状态；
- `m20_warehouse_inspection`：启动组合、契约和记录；
- `m20_scan_navigation`：统一参数。

`src/third_party/SCAN-Planner`、官方 M20 URDF、ONNX 策略、57 维观测和 16 维动作均未
修改。

## 验证

- 五个相关包编译通过；
- 工作空间测试汇总 363 项，0 错误、0 失败、0 跳过；
- 正式 0.90 m 密集场景的同一 F1 左下目标分别在 RViz 和 MuJoCo 完成；
- 两轮运行中 `A-star failed`、`First three control points are in obstacles` 和运行期
  `COLLISION_STOP` 均为 0；
- MuJoCo 使用官方 SDK/ONNX，终点误差 0.060 m；
- 测试中两次调度超时触发执行保持，后续重规划起始速度明确为 `0 0 0`，均自动恢复。

本轮证明原故障链已被切断，但不替代完整 11 步、重复耐久和实机窄通道验收。
