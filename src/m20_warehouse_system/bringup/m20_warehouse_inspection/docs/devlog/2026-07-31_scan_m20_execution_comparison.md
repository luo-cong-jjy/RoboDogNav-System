# 2026-07-31 原版 SCAN 与 M20 执行链对照分析

## 目的与边界

本轮以用户指定的
`src/third_party/SCAN-Planner`
为“原项目”，回答三个问题：

1. 原项目场景 1 是否因为障碍更稀、通道更宽而不易卡住；
2. 原项目 RViz 中的 Go2 是否真的承担了四足动力学执行；
3. 同一类目标下，原执行模型与当前 M20 官方 SDK + MuJoCo 链路的误差出现在哪一层。

本轮没有修改 SCAN 的规划器、优化器、膨胀半径或控制器参数。新增内容仅为只读
对照记录器、场景统计工具、原始 CSV/JSON 数据和本文档。

## 原项目场景 1

原入口使用
`plan_manage/config/simulator.yaml`
中的 Mockamap type-2 随机方柱场景，不是当前仓库系统的 PCD：

```yaml
seed: 127
resolution: 0.1
x_length: 40
y_length: 40
z_length: 5
type: 2
obstacle_number: 500
width_min: 0.2
width_max: 0.8
height_min: 2.0
height_max: 2.0
surface_resolution: 0.05
```

`mockamap/src/maps.cpp` 对每个障碍只采样中心、宽度和高度，没有障碍间距拒绝采样，
也没有外圈围栏。为严格复现 `std::default_random_engine(127)` 的随机序列，新增：

```bash
c++ -std=c++17 -O2 \
  src/m20_warehouse_inspection/tools/scan_vendor_scene1_spacing.cpp \
  -o /tmp/scan_vendor_scene1_spacing
/tmp/scan_vendor_scene1_spacing
```

结果：

| 指标 | 原版 SCAN 场景 1 | 当前 `dense_four_corner` 场景 1 |
|---|---:|---:|
| 地图面积 | 40 m × 40 m | 40 m × 40 m |
| 障碍数量 | 500 | 246（含 6 个固定障碍） |
| 障碍原语面积之和 | 142.114675 m² | 103.693154 m² |
| 重叠障碍对 | 99 | 0 |
| 实体最小正间隙 | 0.003385 m | 0.900236 m |
| 正间隙小于 0.10 m 的障碍对 | 29 | 0 |
| 正间隙小于 0.50 m 的障碍对 | 185 | 0 |
| 正间隙小于 0.90 m 的障碍对 | 429 | 0 |
| 外圈围栏 | 无 | 有，场景连接边保留出口 |

原版地图实际更密、狭小间隙更多。原版能够自由导航，不是因为它给执行模型预留了
更宽通道。大量过窄间隙会在 0.25 m 双圆占据层中自然闭合，规划器选择剩余连通区；
理想执行模型随后非常准确地兑现所选轨迹。

## “Go2 模型”与实际执行模型

原入口会发布 Go2 URDF，用于 RViz 显示和关节动画。其 URDF trunk 碰撞盒为
`0.3762 × 0.0935 × 0.114 m`，但导航位姿不来自该 URDF 的动力学。

真正接收 `/quad_0/cmd_vel` 的是
`plan_manage/src/go2_kinematic_sim.cpp`：

- 100 Hz 平面数值积分；
- `vx / vy / wz` 三轴互相独立；
- 上限为 `0.75 m/s / 0.35 m/s / 1.0 rad/s`；
- 没有质量、惯量、轮地接触、腿部动作、轮速、加速度或倾覆约束；
- 输出速度等于限幅后的输入速度，不存在底层跟踪误差。

所以原版 RViz 中的 Go2 只是可视模型，导航执行本质是一个理想全向平面质点。
它不是 Go2 四足动力学基准，也不能用来证明真实四足或轮足机器能够同样执行
SCAN 的全向命令。

当前 M20 链路则是：

```text
SCAN closed-loop Twist
  -> M20 navigation adapter
  -> double-circle collision guard
  -> safety gate
  -> SDK CmdVelInterface
  -> official M20 policy.onnx
  -> 12 leg position targets + 4 wheel velocity targets
  -> official M20 MJCF + MuJoCo contacts
```

官方 M20 模型证据包括：

- `base_link` 质量 15.882 kg；
- 四个轮子半径 0.09 m；
- 每腿 3 个转动关节加 1 个连续轮关节，共 16 个执行器；
- 轮关节轴固定为机体横向，URDF 中没有车轮转向关节；
- SDK 策略约 50 Hz 推理，腿关节输出位置目标，轮关节输出速度目标；
- ONNX 每帧同时输出全部 16 维动作，不存在“轮子模式/腿模式”的离散输入。

因此腿和轮是否同时参与、参与多少由策略连续输出决定。当前代码里的
`WHEEL_CRUISE`、`COORDINATED_TURN` 只是上层意图标签，不会切换 ONNX 策略。

本地官方部署包的键盘和 `/cmd_vel` 接口给出的命令边界是
`0.7 / 0.5 / 0.7`。同仓库的 `rl_training` M20 rough 配置声明训练命令范围为
`[-2,2] / [-1,1] / [-1,1]`。这些数值能证明策略设计覆盖三轴输入和纯偏航命令，
但不能证明任意三轴组合在当前部署物理链中都稳定；组合速度仍须在 MuJoCo 冷启动
重复实测。

## 动态对照方法

新增只读记录器：

```text
tools/scan_execution_comparison_probe.py
```

它不会重发或改写控制命令，只订阅位姿、地图、原始/候选/安全/SDK Twist、
B-spline、碰撞保持、运动意图和后端故障，并记录：

- 最终误差、路长和直线横向偏差；
- 原始命令横移占比；
- 原始到实际 SDK 命令的矢量差；
- SDK 命令到实测机体速度的矢量差；
- 双圆中心到点云的表面距离；
- B-spline 重规划次数、碰撞保持和 MuJoCo 后端故障。

两组实验分别使用隔离的 ROS Domain 181、182，避免影响用户 Domain 79。
目标形状相同：2.5 m 往返、两个 90° 转弯组合、3 m 往返、6 m 返回。
原版随机地图若目标落在障碍中，记录器在 1.5 m 半径内选择最近安全目标，并在
JSON 中同时保存请求点和实际点。

原版启动：

```bash
source /opt/ros/humble/setup.bash
source src/third_party/SCAN-Planner/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=181
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI=file://$PWD/install/m20_warehouse_inspection/share/\
m20_warehouse_inspection/config/cyclonedds_local.xml

ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false
```

当前 M20 启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=182
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI=file://$PWD/install/m20_warehouse_inspection/share/\
m20_warehouse_inspection/config/cyclonedds_local.xml

ros2 launch m20_warehouse_inspection \
  inspection_mission_mujoco.launch.py \
  use_rviz:=false use_mujoco_viewer:=false
```

## 动态结果

原版 SCAN：

- 6/6 目标成功，总用时 71.46 s；
- 最终误差中位数 0.00366 m；
- 原始命令出现横移的样本占比均值 53.34%；
- 原始命令到实际执行命令的 RMS 差为 0；
- 执行命令到实测速度的 RMS 差均值 0.00637 m/s；
- 共收到 36 条 B-spline，说明原版同样会在线重规划，并非只规划一次。

原版个别绕障段的“点云表面距离减双圆半径”出现负值。该指标基于原始点云最近点，
没有重建 SCAN 栅格的体素中心/去地面细节，因此只作对照诊断，不能据此宣称原版
发生实体碰撞。原版执行器本身也没有物理碰撞模型。

当前 M20 在第一个 2.5 m 无障碍直线目标即失败：

| 指标 | 结果 |
|---|---:|
| 成功 | 否 |
| 故障前行驶距离 | 1.627 m |
| 最终目标误差 | 1.122 m |
| 最大直线横向偏差 | 0.362 m |
| 最小 SCAN 硬边界余量 | 1.224 m |
| 最小独立 guard 余量 | 1.174 m |
| 碰撞停止样本 | 0 |
| 原始命令横移占比 | 28.09% |
| 实际 SDK 横移占比 | 0% |
| 原始到 SDK 命令 RMS 差 | 0.310 m/s |
| SDK 命令到实测速度 RMS 差 | 0.423 m/s |

故障前的关键命令演化为：

```text
约 4.06 s raw       = (0.742, -0.111, -0.211)
约 4.32 s raw       = (0.730, -0.157, -0.294)
约 4.32 s applied   = (0.050,  0.000, -0.366)
约 4.64 s raw       = (0.547, -0.350, -0.965)
约 4.64 s applied   = (0.050,  0.000, -0.650)
约 4.84 s backend   = EXCESSIVE_TILT
```

对应日志：

```text
m20_mujoco_backend: MuJoCo backend fault: EXCESSIVE_TILT
m20_locomotion_manager: backend fault hold: EXCESSIVE_TILT
```

当时最近障碍表面仍有约 1.47 m，远大于 0.25 m SCAN 半径和 0.30 m guard 半径。
这次失败可排除实体障碍、膨胀区和 collision guard 触发。故障发生在 M20
运动执行链，随后后端停止发布有效系统位姿，任务才表现为长时间不动。

## 差异归因

当前最主要的不一致不是规划参数，而是控制对象发生了变化：

1. 原控制器假设 `vy` 可被立即、精确兑现；原版实测约一半运动样本都使用横移。
2. 当前适配器把 `vy` 全部删除，并把
   `atan2(vy, vx)` 乘 1.20 后叠加到原 `wz`。
3. 原 `wz` 已经在跟踪 B-spline 前视切线，再叠加横移方向误差存在重复纠偏。
4. 进入 turn-first 后，前进速度会迅速压到 0.05 m/s，而偏航可升到
   0.65 rad/s，瞬时曲率最高约 13 rad/m。
5. SCAN 看到实际轨迹落后/偏离后继续闭环纠偏和重规划，误差再被适配器放大，
   构成“偏差—大转向—更大偏差”的正反馈。
6. MuJoCo 的倾覆发生在进入障碍安全层之前，因此后续 A* 的
   “robot inside obstacle”是执行偏离的下游结果，不是本次根因。

不能仅凭本轮日志断言 `0.65 rad/s` 单独一定不稳定；真正需要标定的是
`vx / vy / wz / 加速度 / 曲率` 的组合包线，以及策略零命令站立的重复稳定性。

## 下一步优化路线

### 第 1 阶段：先标定官方策略的稳定命令包线

在无障碍平面中按冷启动重复测试，依次增加：

1. 单轴 `vx`、单轴 `vy`、单轴 `wz`；
2. `vx + wz`、`vx + vy`、`vy + wz`；
3. 命令阶跃、斜坡和停止；
4. 90°、180° 转向及不同前进速度下的曲率。

每格至少重复 5 次，记录 roll/pitch 峰值、轮速、腿关节速度、实际速度、
命令跟踪 RMS、停止漂移和故障。生产上限取“连续重复无故障区域”的保守内边界，
而不是直接照抄 SDK 输入 clamp。

### 第 2 阶段：以可实现速度投影替换当前重复纠偏

保留原 SCAN B-spline 与闭环参数，只替换下游适配：

- 不再使用 `raw_wz + 1.20 * atan2(raw_vy, raw_vx)`；
- 将 SCAN 的期望世界速度投影到已标定的 M20 稳定速度集合；
- A/B 比较两种执行策略：
  - 三轴有界直通：保留策略本来支持的少量 `vy`，只做包线和斜率限制；
  - 轮式优先投影：以路径切线和实测机身朝向生成有界 `vx/wz`，只在低速
    横向纠偏时保留小 `vy`；
- `vx` 与 `wz` 使用联合曲率限制，不能分别 clamp 后形成极端组合；
- 转弯前先按可测速度反馈减速，实测速度进入阈值后再放大偏航；
- 进入/退出条件使用实测航向误差和速度，不使用命令值自我判定。

### 第 3 阶段：加入跟踪失配保护

- 连续计算“适配命令—实测速度”和“B-spline—实测位姿”误差；
- 误差超过阈值时先冻结 B-spline 时间、平滑停车，再请求重规划；
- 后端 fault 立即终止当前目标并保存数据，不等待 180 s；
- collision guard 继续检查“适配后的实际候选命令”，不回退检查原始全向命令；
- 根据实测跟踪误差动态增加安全裕度，但不修改 SCAN vendor 膨胀参数。

### 第 4 阶段：逐级验收

```text
无障碍直线
  -> 90°/180° 空旷转向
  -> 单障碍绕行
  -> 0.90 m 窄通道
  -> 同路线六目标
  -> 双场景完整巡检
```

每一级都要求多次冷启动无 `EXCESSIVE_TILT`、无实体接触、无 guard 误停，
并将原始命令、适配命令和实测速度误差作为放行条件。这样可以把“规划可行”和
“M20 可执行”分开验证，后续迁移实机时只需替换反馈与底层 SDK 通道。

## 数据位置

```text
docs/test_data/2026-07-31_scan_motion_comparison/
  original_scan/execution_samples.csv
  original_scan/execution_summary.json
  m20_mujoco/execution_samples.csv
  m20_mujoco/execution_summary.json
```
