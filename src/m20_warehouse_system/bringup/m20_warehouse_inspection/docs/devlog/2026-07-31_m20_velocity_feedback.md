# 2026-07-31 M20 实测机体系速度闭环

## 目标与边界

本轮只修改 M20 运动适配层，不修改 vendored SCAN-Planner、B-spline/A* 参数、地图
膨胀或双圆实体保护半径。目标是在官方策略的命令响应与实测速度之间增加一个受限内环，
改善机器狗对 SCAN 轨迹速度方向的执行一致性。

闭环位于安全预测之前：

```text
/m20/navigation/cmd_vel_raw
  -> RollingNavigationAdapter
  -> measured vx/wz feedback
  -> /m20/navigation/cmd_vel_candidate
  -> collision guard + safety supervisor
  -> /m20/control/cmd_vel_safe
  -> official SDK
```

因此 PI 增加的每一分候选速度仍由碰撞保护按完整双圆扫掠检查。禁止在
`cmd_vel_safe` 之后追加反馈补偿。

## 实现

新增独立纯策略模块 `m20_locomotion_control/velocity_feedback.py`，只闭环：

- `base_link` 前向速度 `linear.x`；
- `base_link` 偏航角速度 `angular.z`。

自主横移仍由既有滚动适配转成航向修正，不增加 `linear.y` 闭环。初始候选参数为：

```yaml
velocity_feedback_linear_kp: 0.30
velocity_feedback_linear_ki: 0.08
velocity_feedback_yaw_kp: 0.20
velocity_feedback_yaw_ki: 0.05
velocity_feedback_linear_error_deadband: 0.03
velocity_feedback_yaw_error_deadband: 0.04
velocity_feedback_linear_integral_limit: 0.20
velocity_feedback_yaw_integral_limit: 0.25
velocity_feedback_linear_correction_limit: 0.10
velocity_feedback_yaw_correction_limit: 0.12
```

控制器同时执行：

1. 软误差死区；
2. 积分与补偿分别限幅；
3. SDK 最终 `0.45 m/s`、`0.65 rad/s` 包络限幅；
4. 条件积分抗饱和；
5. 参考方向改变时清除对应积分；
6. 禁止反馈把正向参考变成反向，或把反向参考变成正向；
7. 零参考立即清除对应积分。

节点订阅 `/m20/sim/body_pose`，只接受 `child_frame_id=base_link` 的有限 twist。
反馈超过 `0.15 s` 未更新、坐标帧错误或数值非有限时，清除积分并退回原开环候选。
订阅锁存的 `/m20/control/execution_hold`；碰撞恢复、任务/切层保持、手动接管和急停
期间都立即清除积分，避免下游停车造成 windup。

`/m20/navigation/velocity_feedback_state` 以 JSON 发布 enabled、active、reason、
measurement_age、参考、实测、误差、补偿、积分和最终候选，供 A/B 探针记录。

## 同步坐标修正

MuJoCo 后端已经把 free-joint 世界系线速度旋转到 `base_link`。本轮复核发现 RViz
平面后端仍把内部 `vx_world/vy_world` 写入 `Odometry.twist`，现改为发布
`vx_body/vy_body`。两个仿真后端和后续实机 profile 因而共享同一 Odometry 契约。

## 自动测试

安装态结果：

| 包 | 测试 | 错误 | 失败 | 跳过 |
|---|---:|---:|---:|---:|
| `m20_locomotion_control` | 61 | 0 | 0 | 0 |
| `m20_warehouse_sim` | 21 | 0 | 0 | 0 |
| `m20_warehouse_inspection` | 180 | 0 | 0 | 1 |

新增纯策略测试覆盖误差死区、正向/偏航补偿、双重限幅、禁止反转、饱和抗积分、
参考换向清积分、零参考复位、横移透传和非有限反馈降级。启动契约测试锁定反馈仍位于
碰撞保护之前，并锁定 Odometry、execution-hold 和显式 A/B 参数。

## MuJoCo A/B 结果

同任务采用 `single_obstacle_direct_bypass`，每组独立冷启动；SCAN、地图、碰撞保护、
任务目标和官方 M20 SDK 策略保持一致，只切换
`velocity_feedback_enabled:=false/true`。

| 指标 | 开环 | 闭环 | 变化 |
|---|---:|---:|---:|
| 任务 | 完成 | 完成 | 一致 |
| 用时 | `28.916 s` | `27.495 s` | `-4.91%` |
| 终点误差 | `0.0322 m` | `0.1067 m` | `+0.0745 m` |
| 前向速度跟踪 RMS | `0.1160 m/s` | `0.0997 m/s` | `-14.06%` |
| 偏航速度跟踪 RMS | `0.1041 rad/s` | `0.0964 rad/s` | `-7.40%` |
| 最小 guard 净空 | `0.2485 m` | `0.1977 m` | `-0.0509 m` |
| 门口采样 guard 净空 | `0.4288 m` | `0.2940 m` | `-0.1348 m` |
| 预测停车事件 | 1 | 0 | `-1` |
| 转向恢复事件 | 1 | 0 | `-1` |
| 最长有命令静止 | `0.4650 s` | `0.3620 s` | `-22.14%` |
| 当前包络侵入 | 0 | 0 | 一致 |
| MuJoCo 障碍接触 | 0 | 0 | 一致 |

闭环有效样本为 526，最大前向/偏航补偿分别为 `0.0558 m/s` 和
`0.0564 rad/s`，均明显小于配置的 `0.10 m/s`、`0.12 rad/s` 补偿上限。
闭环全程没有当前包络侵入、实体接触、非有限反馈或错误坐标帧。

证据：

```text
docs/test_data/2026-07-31_m20_command_envelope/
  velocity_feedback_ab/
    open_loop/
      clearance_samples.csv
      clearance_summary.json
    closed_loop/
      clearance_samples.csv
      clearance_summary.json
```

## 放行结论

闭环已经证明可以工作，并改善速度跟踪、任务用时和停车恢复；但单次结果的最小 guard
净空比开环少 `50.9 mm`，终点误差增加 `74.5 mm`。两项仍满足本任务的安全和
`0.20 m` 到点容差，但没有满足本轮预先规定的“guard 净空不降低”默认放行条件。
考虑 SCAN 每次冷启动的局部轨迹并非严格确定，不能仅凭一对样本把净空差异完全归因于
速度反馈，也不能忽略它。

因此 `velocity_feedback_enabled` 保持默认 `false`，闭环作为显式 A/B 选项保留：

```bash
ros2 launch m20_warehouse_inspection inspection_mission_mujoco.launch.py \
  velocity_feedback_enabled:=true
```

## 重复冷启动与六目标放行门

后续完成三次单障碍闭环独立冷启动。3/3 任务成功，当前包络侵入和实体接触都为零；
但最小 guard 净空分别为 `0.0746 / 0.1336 / 0.1932 m`，其中第一轮触发三次
预测停车，总用时增加到 `42.371 s`。这证明闭环硬安全降级有效，但净空和任务稳定性
尚不具备默认放行重复性。

同配置六目标冷启动结果：

- 开环 6/6 到达，总用时 `160.49 s`；
- 闭环前四个目标完成，第五个 180°返回在 `90 s` 超时，第六目标未执行；
- 开环第五段也有约 `50.8 s` 碰撞停车，闭环增加到约 `79.1 s`；
- 两组均无实体障碍接触，现场碰撞诊断为预测足迹停车；
- 闭环探针结束后系统继续恢复并最终抵达目标，但超过验收门限。

失败链路是 SCAN 纯偏航请求进入滚动 M20 运动投影后，预测保护触发后退恢复；恢复轨迹
又持续落入预测占据，机器人被反复带离目标。反馈在 execution hold 期间正确清积分，
因此不是积分 windup。单次六目标 A/B 不能完全排除 SCAN 冷启动轨迹差异，但闭环组
恢复样本更多并超过验收超时，已经不具备默认放行条件。

最终维持 `velocity_feedback_enabled: false`。下一阶段先修复纯偏航投影与有界恢复
状态机，再重新进行开环六目标基线；在开环零长期恢复之前，不继续调高或默认启用 PI。
完整数据和逐目标判定见
`docs/test_reports/2026-07-31_m20_velocity_feedback_ab.md`。

## 2026-08-01 后续结果

纯偏航投影、有界恢复和双向直线跟踪已完成后续修复。新基线六目标开环与受门控闭环
均达到 `6/6`，但闭环单障碍重复样本仍有明显最小净空波动，因此默认关闭结论不变。
最新实现和数据见
`docs/test_reports/2026-08-01_m20_velocity_feedback_requalification.md`。
