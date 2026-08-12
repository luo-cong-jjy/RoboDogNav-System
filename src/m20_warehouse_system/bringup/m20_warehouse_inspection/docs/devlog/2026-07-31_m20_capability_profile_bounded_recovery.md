# 2026-07-31 M20 平台能力配置与有界恢复

## 本轮目标

本轮只优化 SCAN 规划器之后的 M20 执行边界，不改
`src/third_party/SCAN-Planner` 的地图、A*、B-spline、重规划频率或可视化：

1. 将导航适配器、碰撞保护和官方 SDK 桥重复维护的速度/转向参数收敛为一个
   版本化平台能力配置；
2. 删除 native-SCAN 碰撞保护入口中遗留的理想质点限速；
3. 将碰撞恢复限定为短时脱困动作，禁止它在路径持续不可执行时无限代替导航器；
4. 针对 M20 不支持可靠零半径偏航的能力边界，让完整系统在轨迹切向位于机身后方
   时沿同一条 B-spline 倒车跟踪，而不是把 180° 回程投影成向前滚动掉头。

## 单一能力配置

新增：

```text
m20_locomotion_control/config/m20_policy_v1_capabilities.yaml
```

该文件保存当前官方 M20 ONNX 策略已验证的：

- 机身外廓 `0.82 × 0.51 × 0.57 m`；
- SDK 命令上限 `0.45 m/s / 0.20 m/s / 0.65 rad/s`；
- 不允许自主导航把 `vy` 当作全向横移；
- 不声明可靠的零半径偏航能力；
- 稳定滚动转向区间 `0.35–0.45 m/s`；
- 方向相关侧向漂移、反向死区补偿和跟踪整形参数；
- 允许倒车跟踪及进入/退出角度滞回；
- 碰撞恢复速度、扫掠时间及退出预算。

加载器在节点启动前校验 schema、数值有限性、正值约束、转向速度区间和恢复速度
是否越过 SDK 包线。非法配置直接终止启动，不能以一组局部默认值静默运行。

由能力配置计算：

```text
最小中心线转弯半径 = 0.35 / 0.65 = 0.538 m
外侧机身角点扫掠半径
  = hypot(0.538 + 0.51 / 2, 0.82 / 2)
  ≈ 0.893 m
```

`0.893 m` 是空间布局与诊断指标，不直接作为圆形障碍膨胀半径。运行时碰撞判断仍
采用前/后双圆、预测姿态和方向相关漂移扫掠，避免用一个大外接圆封死可以直行通过
的 `0.90 m` 通道。规划净空与占据栅格分辨率仍由 `m20_scan_navigation` 的独立
clearance profile 管理。

## 参数注入链

主启动现在只解析一次能力配置，并把对应子集注入三个消费者：

```text
m20_policy_v1_capabilities.yaml
  ├─ intent_parameters
  │    ├─ m20_navigation_adapter（安全预测之前）
  │    └─ m20_locomotion_manager（SDK 最终门）
  ├─ collision_guard_parameters
  │    └─ m20_collision_guard（速度、漂移、恢复）
  ├─ controller_parameters
  │    └─ closed_loop_controller（M20 前向/反向轨迹跟踪选择）
  └─ sdk_parameters
       └─ m20_sdk_deploy/rl_deploy_cmdvel（官方策略输入限幅）
```

`sdk_locomotion.yaml` 只保留话题、节点角色和可选速度闭环设置；
`collision_guard*.yaml` 只保留地图、双圆几何和时域安全设置。原 native-SCAN 启动中
硬编码的 `0.75 / 0.35 / 1.0` 质点上限已删除，碰撞预测与真正送入 M20 的 candidate
现在共享 `0.45 / 0.20 / 0.65` 包线。

主启动可显式替换能力文件：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_mujoco.launch.py \
  locomotion_capability_config:=/absolute/path/to/profile.yaml
```

默认启动指令和两终端操作流程没有变化。

## 有界碰撞恢复

恢复动作仍必须满足以下既有前提：

- 当前前/后双圆没有进入阻塞栅格；
- 原 candidate 只是在预测时域内将发生碰撞；
- 完整恢复扫掠在 nominal、实测漂移和反向不确定性三种模型下全部为空；
- 纯偏航、反向导航请求或过小偏航请求不能触发滚动恢复。

本轮增加同一次障碍事件内不可重置的预算：

| 条件 | 默认值 | 结果 |
|---|---:|---|
| 最大连续恢复时间 | `6.0 s` | `TIME_LIMIT` 后硬停止 |
| 最大离起始恢复点位移 | `0.75 m` | `DISTANCE_LIMIT` 后硬停止 |
| 无进展检测窗口 | `1.50 s` | 未移动 `0.03 m` 时 `NO_PROGRESS` |
| 恢复动作释放 | 连续 clear `0.30 s` | 停止当前脱困命令，但保留本次预算 |
| 预算重新许可 | 连续 clear `1.50 s` | 清空耗尽锁存，下一障碍事件可重新恢复 |

切换正/反向恢复或左右转向不会重置预算。耗尽后
`/m20/control/collision_recovery_available=false`，安全 supervisor 保持零命令，
诊断话题输出 `RECOVERY_BUDGET_EXHAUSTED` 与具体原因。这样即使 SCAN 在障碍边界
持续给出同一不可执行 candidate，恢复也不会无限驱动 M20 漂移。

短暂 clear 不再被当作新障碍事件。动态测试首先复现了旧行为：同一个第 5 回程段
通过 `0.30 s` clear 反复重置预算，累计出现 17 次恢复、587 个停车样本。加入
`1.50 s` 重新许可滞回后，同一路段在 `0.75 m` 位移上限处正确锁存
`DISTANCE_LIMIT`，从而暴露了真正问题不是恢复预算，而是 180° 轨迹的执行投影。

## M20 双向轨迹执行

原 SCAN 控制器在目标切向位于机身正后方时先输出
`(vx, vy, wz)=(0, 0, ±1)` 进行理想原地对正。M20 能力 profile 明确
`supports_zero_radius_yaw=false`，导航适配器只能把该命令变为已验证的最低稳定
滚动弧 `(vx≈0.35, wz=±0.65)`；在第 5 回程段，这个向前扫掠恰好驶向障碍，随后
触发有界恢复。

完整 M20 集成入口现在启用以下执行规则：

- B-spline 前向切向与机身航向误差大于 `2.10 rad` 时进入 `REVERSE`；
- 误差回到 `1.75 rad` 内且至少保持 `0.80 s` 后才退出，避免重规划时前后抖动；
- `REVERSE` 只把控制器的跟踪航向旋转 `pi`，世界系 B-spline 位置、速度、A*、
  B-spline 和重规划周期均不改变；
- 世界系路径速度转换到当前机体系后自然得到负 `vx`，直接使用官方 SDK 已有的
  reverse 命令能力，不添加横移或位姿传送；
- `/planning/tracking_direction` 发布 `FORWARD/REVERSE`，用于运行审计。

该能力默认只由完整 M20 launch 从 `m20_policy_v1` 注入。独立
`m20_scan_navigation/f1_scan.launch.py` 的默认值仍是 `false`，因此原 SCAN
场景、参数和执行效果不会被平台适配逻辑静默改变。

## 验证

源代码阶段验证：

```text
Python 静态编译：通过
相关 pytest：通过
ament_flake8：14 files, no problems
colcon build：m20_locomotion_control、m20_scan_planner、
              m20_scan_navigation、m20_warehouse_inspection 四包通过
工作区累计 colcon test-result：464 tests，0 errors，0 failures，1 skipped
安装空间 ros2 launch --show-args：通过，默认能力文件解析为 m20_policy_v1
```

单元测试覆盖配置一致注入、非法转向区间拒绝、最小转弯/扫掠半径、无进展退出、
绝对时间退出、最大位移退出、重新许可滞回、能力配置控制器注入和 standalone 默认
关闭双向跟踪。

## MuJoCo 动态验收

单障碍原生 SCAN 绕行在官方 SDK + MuJoCo 冷启动中成功：`26.09 s` 到达、终点
误差 `0.0875 m`、SCAN 硬边界最小余量 `0.1929 m`、独立 guard 最小余量
`0.1429 m`，无预测恢复、当前足迹事件、预算耗尽或实体障碍接触。

同一组六目标在双向跟踪启用后达到 `6/6`，总用时 `102.72 s`，最大终点误差
`0.1607 m`，所有目标的碰撞停车、恢复、预算耗尽和实体障碍接触均为 0。两个
180° 回程段分别为：

| 路段 | 用时 | 横向误差 | 路径长度比 | REVERSE 占比 |
|---|---:|---:|---:|---:|
| 2.5 m 回程 | `12.87 s` | `0.0084 m` | `0.982` | `76.7%` |
| 3.0 m 回程 | `16.32 s` | `0.0051 m` | `0.986` | `77.2%` |

作为对照，修复前第 5 段需要 `49.05 s`、行驶 `9.13 m`、发生 17 次恢复；严格
保持恢复预算后则在 `90 s` 超时并锁存 `DISTANCE_LIMIT`。新实现同段
`16.32 s` 完成且没有任何恢复，证明修复作用于运动执行方向，而不是放宽膨胀层或
绕过安全保护。

完整原始数据与逐项结论见
`docs/test_reports/2026-07-31_m20_bidirectional_tracking_bounded_recovery.md`。
下一步是在该稳定开环基线上重新做速度闭环 A/B，再运行默认双场景完整巡检。真机
接入时应新增实际交付固件的能力 profile/backend，不复制或修改 SCAN 参数。
