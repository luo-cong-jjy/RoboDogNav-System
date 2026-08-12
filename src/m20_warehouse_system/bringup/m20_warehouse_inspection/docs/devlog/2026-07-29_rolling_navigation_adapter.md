# M20自动导航滚动适配开发记录

日期：2026-07-29

> 2026-07-30 修订：本文记录首次 A/B 版本，当时适配位于 safety 之后。绕障回归发现
> 这种位置会让碰撞预测和实际执行使用不同命令；当前实现已把同一适配器前移到
> `/m20/navigation/cmd_vel_candidate`，末端
> `rolling_navigation_enabled=false`。详见
> [2026-07-30_candidate_safety_freeze.md](2026-07-30_candidate_safety_freeze.md)。

## 目标与边界

基线测试证明原版SCAN能够完成自由导航，但63.81%的运动采样被旧速度接口归为
`LATERAL_MANEUVER`。普通路径对齐因此长期保留机体系侧向速度，官方轮腿策略会在
低速前进时产生较明显的腿部协调动作。

本次只修改SCAN安全速度与官方M20 SDK之间的项目自有适配层：

- 不修改SCAN规划、B样条、点云、占据或可视化；
- 不修改官方ONNX、57维观测、16维动作和关节标定；
- 不锁死腿关节，仍由官方策略协调12个腿关节和4个轮关节；
- 不削弱急停、碰撞、楼层切换、任务保持、后端故障和超时门控；
- 保留人工调试和恢复所需的明确横移能力。

## 实现结构

`m20_locomotion_manager`新增对锁存
`/m20/control/safety_state`的订阅，并按命令来源选择策略：

```text
NAVIGATION
  -> RollingNavigationAdapter
  -> vx / 0 / wz
  -> official ONNX

MANUAL
  -> existing constrain_for_intent
  -> lateral motion remains available

hold / fault / timeout
  -> immediate zero and adapter reset
```

滚动适配器位于
`m20_locomotion_control/m20_locomotion_control/motion_intent.py`，每个控制周期
依次执行：

1. 按SDK速度包络限制`vx/vy/wz`，非有限输入清零；
2. 用`atan2(vy, max(abs(vx), speed_floor))`计算机体系行进方向误差；
3. 把方向误差按增益叠加到偏航命令，自动导航输出的`vy`固定为零；
4. 大方向误差进入`COORDINATED_TURN`，前进速度限制为0.12 m/s；
5. 使用不同的进入/退出阈值保持模式迟滞，避免阈值附近来回切换；
6. 普通`WHEEL_CRUISE`对小偏航使用连续软死区和一阶低通；
7. 对前进和偏航输出施加变化率限制；
8. 零指令、硬停止、后端保持或命令超时立即清零并重置滤波状态。

当前默认参数集中在
`m20_locomotion_control/config/sdk_locomotion.yaml`：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `rolling_navigation_enabled` | `true` | 仅对自动导航启用滚动适配 |
| `allow_manual_lateral` | `true` | 保留人工横移 |
| `course_yaw_gain` | 0.80 | 侧向方向误差到偏航的增益 |
| `turn_course_enter` | 0.25 rad | 协调转向进入方向阈值 |
| `turn_course_exit` | 0.08 rad | 协调转向退出方向阈值 |
| `turn_yaw_exit` | 0.12 rad/s | 协调转向退出偏航阈值 |
| `cruise_yaw_deadband` | 0.04 rad/s | 巡航偏航软死区 |
| `cruise_yaw_filter_time_constant` | 0.12 s | 巡航偏航低通时间常数 |
| `output_linear_accel` | 1.0 m/s² | 适配层线速度变化率 |
| `output_yaw_accel` | 1.2 rad/s² | 适配层偏航变化率 |

`rolling_navigation_enabled: false`可恢复旧的统一意图约束，便于后续对比与现场回退。

## 测试设计

验收条件在修改前冻结：

- 与基线相同的六个自由导航目标必须6/6完成；
- 每个目标最终误差不大于0.25 m；
- `collision_stop`样本必须为0；
- 自动`LATERAL_MANEUVER`应从63.81%降至接近0；
- 两个180°目标的最大横向偏离均应低于各自基线；
- 首段无障碍直线的小偏航和明显腿部动作应下降。

新增纯逻辑测试覆盖侧向到偏航转换、转向迟滞、立即零停、自动零横移以及原人工横移
策略。两个相关包构建后测试结果分别为30项和135项，共165项，0失败。

## 结果与判断

相同0.90 m默认场景、同一官方ONNX、同一MuJoCo后端和同一六目标路线闭环重跑后：

- 6/6目标成功，最大终点误差0.161 m；
- 自动`LATERAL_MANEUVER`为0%，人工横移接口保留；
- 总耗时175.59 s降至101.59 s；
- 平均终点误差0.162 m降至0.092 m；
- 平均最大横向偏离0.274 m降至0.193 m；
- 两个180°横向偏离由0.611/0.564 m降至0.530/0.064 m；
- 首段直线SDK偏航RMS由0.079降至0.044 rad/s；
- 首段直线腿部最大关节速度RMS由1.274降至0.319 rad/s；
- 模式切换次数由50降至25；
- `collision_stop`仍为0，最大接触数量仍为8。

轮腿同时参与没有也不应降为0。官方策略仍可在滚动时调整腿部以维持姿态；本次减少
的是导航层不必要的横移和小偏航请求，而不是改变策略动作空间。

两个90°目标的最大中心线偏离从0.099/0.094 m上升到0.173/0.185 m，但均低于
0.19 m、无碰撞停止，且总时间、终点精度、180°扫掠和直线平顺性明显改善。当前
参数作为默认值放行，后续完整11步任务与实机窄通道验收继续保留。
