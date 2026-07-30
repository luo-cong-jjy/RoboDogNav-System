# 2026-07-30 M20 航向跟随与预测停车恢复

## 问题

MuJoCo 实测中，SCAN 已生成可行 B-spline，但 M20 的机身朝向偶尔落后于轨迹切线。
旧控制器在航向误差小于 `0.80 rad` 时仍允许平移，滚动适配器在高曲率状态也可保留
`0.12 m/s` 前进速度。对质点而言可行的轨迹因此可能被有长宽的轮足机身“切角”执行，
随后独立双圆保护器触发 `PREDICTED_FOOTPRINT`，SCAN 又可能报告：

```text
A-star failed
First three control points are in obstacles
Failed to generate rebound direction
```

已有日志和 MuJoCo 接触统计表明，所复现的卡住首先是预测保护停车，不是已撞上障碍物。
但如果控制器继续推进，机身确实有进入当前保护区的风险，因此不能通过继续缩小膨胀阈值
来处理。

## 设计边界

本轮保持原版 SCAN 的点云、GridMap、A-star、B-spline 优化和 RViz 可视化算法不变。
改动只位于以下三个可替换边界：

1. SCAN B-spline 之后的闭环执行控制器；
2. 安全预测之前的 M20 SDK 速度适配器；
3. 独立于 SCAN 的 fail-closed 碰撞保护器。

原始单场景 `f1_scan.launch.py` 仍默认加载 vendor 控制参数。仓库完整系统入口才叠加
`scan_m20_physical_controller.yaml`，便于后续继续做原版对照和实机移植。

## 航向对正控制

闭环控制器新增带迟滞的航向状态：

- `|heading_error| <= 0.15 rad`：正常平移；
- `0.15 < |heading_error| < 0.55 rad`：按误差线性降低平移速度；
- `|heading_error| >= 0.55 rad`：进入纯偏航对正，冻结 B-spline 执行时钟；
- 已进入对正后，至少保持 `0.40 s`，且误差降到 `0.20 rad` 以下才恢复平移。

迟滞避免重规划造成“前进—转向”快速抖动；冻结轨迹时钟避免机器人原地对正期间
B-spline 时间继续前进。新增诊断：

- `/m20/navigation/heading_error`；
- `/m20/navigation/heading_aligning`。

## SDK 速度适配

自动导航仍是滚动优先，但加强了轨迹横向误差到偏航的修正：

```yaml
course_yaw_gain: 1.20
cruise_yaw_deadband: 0.02
turn_course_enter: 0.25
turn_course_exit: 0.08
turn_yaw_exit: 0.20
turn_min_hold_sec: 0.50
turn_max_forward: 0.05
```

高曲率转向最多保留 `0.05 m/s` 的低速爬行；闭环控制器发出纯偏航命令时，适配器无条件
清零平移。轮腿何时具体发力仍由官方 ONNX 策略根据连续 `vx/vy/wz` 和本体状态决定，
这里没有伪造轮式/四足离散步态开关。

曾测试把 `turn_max_forward` 完全设为零。它能提高单障碍绕行净空，但六目标自由导航中
出现长期转向模式和最终目标超时，因此没有作为默认参数。最终值保留少量可控爬行，并将
转向退出偏航阈值放宽到 `0.20 rad`。

## 足迹与规划包络

正式 `conservative` 参数改为：

```yaml
grid_map.double_cylinder_radius: 0.30
optimization.dist0: 0.15
footprint_radius: 0.25
safety_margin: 0.05
lookahead_sec: 0.70
```

这把 `0.05 m` 从优化器软距离移入 SCAN 硬规划足迹：

- SCAN 硬足迹：`0.30 m`；
- 独立保护足迹：`0.25 + 0.05 = 0.30 m`；
- SCAN 名义总单侧包络仍为 `0.30 + 0.15 = 0.45 m`；
- `0.90 m` 最小场景间距保持不变；
- 可选栅格路线膨胀仍为 `0.45 m`。

SCAN 栅格分辨率为 `0.05 m`。实测 `0.32 m` 和 `0.35 m` 都量化到下一圈栅格并封闭
`0.90 m` 基准通道，因此 `0.30 m` 是当前通道宽度下可使用的最大硬足迹，不再继续增大。

## 预测停车恢复

原逻辑一旦未来预测样本碰撞就完全清零。轨迹时钟同时被冻结后，机器人可能永远保持同一
朝向，形成预测停车死锁。新逻辑只对这种情形增加受保护的纯旋转：

1. 当前样本必须完全安全；
2. 原前进命令的未来样本触发 `PREDICTED_FOOTPRINT`；
3. 保护器用同一双圆足迹、同一占据图和同一预测时域验证纯偏航；
4. 请求方向不可行时再验证反方向；
5. 只有整个旋转预测安全才发布零平移的恢复命令。

`CURRENT_FOOTPRINT`、地图未就绪、急停、切图保持、任务保持、里程计超时等状态从不允许
恢复运动，仍然立即输出全零。新增接口：

- `/m20/control/collision_recovery_available`；
- `/m20/control/collision_recovery_cmd`；
- safety state `COLLISION_RECOVERY`。

## 可观测性与测试工具

`navigation_motion_probe.py` 升级到 schema 2，新增：

- 控制器航向误差均值、P95 和最大值；
- 航向对正占比和进入次数；
- 运动模式切换次数；
- MuJoCo 障碍接触事件与峰值力；
- 预测纯旋转恢复事件数。

完整测试数据和否决实验见
`docs/test_reports/2026-07-30_heading_alignment_control.md`。

