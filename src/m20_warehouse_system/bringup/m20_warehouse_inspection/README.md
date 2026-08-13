# M20 平面双区域仓库巡检系统

`m20_warehouse_inspection` 是云深处 M20 双层仓库巡检项目的新集成边界。

当前确认的第一阶段仿真假设是：

- 不模拟真实楼层高度、楼梯、坡道或爬楼动作。
- 在 RViz 的同一个 `z=0` 平面中相邻放置两块 40 m × 40 m 区域：F1 为
  `x=[-40,0]`，F2 为 `x=[0,40]`。
- 两区共用原点 `(0,0)` 作为逻辑楼层切换点；相邻围栏各留一个 4 m 宽门洞。
- 启动时同时显示 F1 和 F2；规划器始终只接收当前 active floor。
- M20 必须先自主导航到原点并停车，系统才原地切换 active PCD；默认流程
  不调用 `/m20/sim/set_pose`，机器狗位姿连续。
- F2 复用 F1 的内部障碍布局，但相邻边门洞方向相反，因此两份资产的内部完全一致、
  边界墙面有意不同。
- 默认 RViz 保留 SCAN 原版 active-floor 显示，并以蓝灰色显示未激活场景。

完整设计与分阶段验收标准见 [docs/system_plan.md](docs/system_plan.md)。

## 当前进度

阶段 0～5、0.7.0～1.3.0 校正已完成；RViz 1.3.0 稳定配置、地图和官方模型已通过
`config/rviz_v1_baseline.yaml` 冻结：

- 建立独立 ROS 2 包边界。
- 固化平面双区域、楼层、电梯和切图事务配置。
- 固化第三方依赖来源与开发记录约定。
- 实现配置 schema 与跨字段校验。
- 确定性生成 F1 资产，并按 `F2.replica_of: F1` 复用内部障碍；相邻侧边界分别生成
  对向门洞。
- 实现 all-floors 总览和 initial active-floor 地图服务器。
- 提供 RViz 双区域静态基线。
- 包装云深处官方 M20 URDF 与 17 个官方 STL；RViz 基线直接读取官方运动树，不附加
  雷达、IMU 或其他可视链接。
- 实现不依赖 Gazebo 的 RViz 平面运动学后端。
- 将 SCAN local sensing、重规划、B 样条和闭环控制接入 F1。
- 将第三方首场景的参数、CPU `pcl_render_node`、控制器、标准话题图和 RViz 布局
  整体接入；`m20_scan_planner` 是带楼层 reset 扩展的唯一 SCAN 核心。
- 已移除项目新增的离线 PGM/YAML、静态占据栅格 A* 和顺序短子目标；日常手动与
  自动任务都只使用原生 SCAN 的实时三维 GridMap、rebound、B-spline 和可视化。
- 建立 fail-closed 安全 supervisor 和独立前视碰撞保护。
- 完成 F1 有障碍路线、终点停车、动态急停和桌面 RViz 渲染验收。
- 增加独立的 typed interface 和 generation-aware local sensing 包。
- 实现基于 compare-and-swap 的 active PCD 原子切换。
- 实现 SCAN GridMap、目标和轨迹的受控 reset。
- 实现共享原点无位姿传送的双向 `/m20/floor_switch` Action；旧的分离区域 profile
  仍可显式启用仿真传送，但不属于当前默认流程。
- 完成 F1→F2→F1 双向运行时验收；新楼层感知范围不存在旧楼层点云残留。
- 实现 `/m20/navigation/navigate` typed Action gateway，把目标与
  `floor_id + map_generation` 绑定，并提供取消、超时、路线失败和最终距离结果。
- 实现 `/m20/mission/run` 自动任务执行器，按配置串联
  F1_A、F1_B、E1:F1→F2、F2_A、F2_B、F2_RETURN_VIA_A、F2_TERMINAL。
- 增加与已验证基线隔离的 `dense_four_corner` 场景副本：每场景 246 个障碍物
  （240 个确定性随机障碍物和 6 个过道固定障碍物），独立障碍物边界间距不小于
  0.9 m；两个场景均按左下、右下、右上、左上巡检，F2 完成后反向切回 F1 并
  返回全流程起点。
- 增加不修改冻结基线的 `route_challenge` 测试候选：每场景 252 个障碍物，其中
  12 个固定障碍物进入名义巡检线，用于后续 SCAN 绕障与压力回归。
- 所有切换步骤都先导航到共享原点 `(0,0)`，停车后才切换 active map。
- 人工目标和自动完整任务共用原版 `/move_base_simple/goal -> SCAN -> B-spline ->
  closed-loop controller` 链路。
- 完整系统与独立启动均使用原项目 SCAN 参数：`0.25 m` 硬规划双圆半径和
  `0.20 m` 优化器软距离。额外 `0.05 m` 执行余量只属于下游
  `collision_guard`，不再反向修改 SCAN 的路径搜索和 B-spline 参数。
- RViz 的 active floor 恢复原项目 `AxisColor`，inactive floor 使用蓝灰色；不再把
  同一 active PCD 通过两个图层重叠绘制，因此移动视角不会发生黄/粉色深度竞争。
- 实现任务 pause、resume、stop 和 fault retry；任务暂停或故障使用独立
  `mission_hold` 立即进入安全零速。
- 完成含中途暂停/恢复的旧五步端到端基线验收：F1→原点→原地切图→F2→原点，
  generation 2，切换位姿跳变 `0.000 m`，终点误差 `0.142 m`。
- 固定十个项目包、22 包依赖闭包、三个第三方 revision，以及可校验的 SCAN/SDK
  适配补丁。
- `m20_warehouse_inspection` 是唯一用户入口；完整模块清单、默认隔离的旧包及恢复规则见
  [docs/package_inventory.md](docs/package_inventory.md)。
- 在只加载 `/opt/ros/humble` 的全新工作空间完成独立构建与 209 项测试。
- 完成 20/20 次 F1↔F2 交替切图，最终回到 F1 generation 21，保持速度违规为 0。
- 完成 10/10 次生产速度五步任务，共 50/50 step，最终为 F2 generation 20。
- 完成非法切层、旧代次、导航取消、任务停止、FAULT_HOLD 和 retry-current 故障矩阵。
- 增加 launch 子进程 RSS 预热/稳态趋势、自动回归图关闭和 Humble/Foxy 兼容清单。
- 新增隔离的 `m20_mujoco_backend`：官方 M20 16 执行器 MJCF 直接执行官方 ONNX
  运控输出，不再用平面质点/四轮车近似作为完整联仿后端。
- 从当前 system YAML 所引用的地图 JSON 同源生成 MuJoCo 碰撞世界；0.90 m 默认高密度
  profile 一次加载两区共 492 个障碍实体和保留共享门洞的 8 段去重围栏。
- 完整联仿保持原版 SCAN 点云、规划、B 样条、可视化、typed 任务和原地切图机制；
只替换速度之后的运控/物理执行以及 `/m20/sim/body_pose` 来源。
- 后端在官方 SDK 完成站立且姿态稳定前保持 `backend_ready=false`，不向 SCAN 开放
  body pose/TF；故障或后端未就绪时速度适配器立即进入 hold。
- 新增版本化 `m20_policy_v1` 平台能力配置，使导航适配、碰撞保护、最终 SDK 门和
  官方策略桥共享同一 M20 包线；恢复动作采用 `6 s / 0.75 m` 有界预算及连续
  `1.50 s` clear 后才重新许可。
- 完整 M20 系统根据 B-spline 切向在 FORWARD/REVERSE 间滞回选择；180° 回程直接
  沿原轨迹倒车，不再将原 SCAN 的理想纯偏航请求投影为向前滚动掉头。独立 SCAN
  启动默认关闭该平台适配。

阶段 5 记录见
[docs/devlog/2026-07-27_phase5.md](docs/devlog/2026-07-27_phase5.md)，验收摘要见
[docs/test_reports/2026-07-27_phase5.md](docs/test_reports/2026-07-27_phase5.md)。
模型与 SCAN 显示/链路校正见
[docs/devlog/2026-07-27_model_scan_correction.md](docs/devlog/2026-07-27_model_scan_correction.md)。
任务终点、长路线、配色和围栏语义校正见
[docs/devlog/2026-07-27_mission_visualization_correction.md](docs/devlog/2026-07-27_mission_visualization_correction.md)，
对应运行结果见
[docs/test_reports/2026-07-27_mission_visualization_correction.md](docs/test_reports/2026-07-27_mission_visualization_correction.md)。
SCAN 源码、参数、默认链路和 RViz 显示逐项审计见
[docs/devlog/2026-07-27_scan_upstream_consistency.md](docs/devlog/2026-07-27_scan_upstream_consistency.md)，
对应测试见
[docs/test_reports/2026-07-27_scan_upstream_consistency.md](docs/test_reports/2026-07-27_scan_upstream_consistency.md)。
首场景整体移植记录见
[docs/devlog/2026-07-27_scan_vendor_first_scene_transplant.md](docs/devlog/2026-07-27_scan_vendor_first_scene_transplant.md)，
运行验收见
[docs/test_reports/2026-07-27_scan_vendor_first_scene_transplant.md](docs/test_reports/2026-07-27_scan_vendor_first_scene_transplant.md)。
场景 1 严格复制、巡检与事务切图融合记录见
[docs/devlog/2026-07-28_scene1_replica_mission_integration.md](docs/devlog/2026-07-28_scene1_replica_mission_integration.md)，
对应验收见
[docs/test_reports/2026-07-28_scene1_replica_mission_integration.md](docs/test_reports/2026-07-28_scene1_replica_mission_integration.md)。
相邻双场景、原点无瞬移切图的最新记录见
[docs/devlog/2026-07-28_connected_origin_map_switch.md](docs/devlog/2026-07-28_connected_origin_map_switch.md)，
对应验收见
[docs/test_reports/2026-07-28_connected_origin_map_switch.md](docs/test_reports/2026-07-28_connected_origin_map_switch.md)。
F2 双点覆盖与返回起点的扩展记录见
[docs/devlog/2026-07-28_f2_full_coverage_return.md](docs/devlog/2026-07-28_f2_full_coverage_return.md)，
对应测试状态见
[docs/test_reports/2026-07-28_f2_full_coverage_return.md](docs/test_reports/2026-07-28_f2_full_coverage_return.md)。
高密度四角巡检副本的参数、生成规则和路线记录见
[docs/devlog/2026-07-28_dense_four_corner_patrol.md](docs/devlog/2026-07-28_dense_four_corner_patrol.md)，
对应测试状态见
[docs/test_reports/2026-07-28_dense_four_corner_patrol.md](docs/test_reports/2026-07-28_dense_four_corner_patrol.md)。
正式默认间距调整至0.90 m的配置、资产和冻结哈希记录见
[docs/devlog/2026-07-29_dense_default_spacing_090.md](docs/devlog/2026-07-29_dense_default_spacing_090.md)，
对应静态与包级回归见
[docs/test_reports/2026-07-29_dense_default_spacing_090.md](docs/test_reports/2026-07-29_dense_default_spacing_090.md)。
RViz 1.3.0 冻结和路线受扰动场景记录见
[docs/devlog/2026-07-28_rviz_v1_freeze_route_challenge.md](docs/devlog/2026-07-28_rviz_v1_freeze_route_challenge.md)，
对应静态验收见
[docs/test_reports/2026-07-28_rviz_v1_freeze_route_challenge.md](docs/test_reports/2026-07-28_rviz_v1_freeze_route_challenge.md)。
SDK 运控输入、轮腿模式边界、Gazebo 可行性和分阶段验收见
[docs/devlog/2026-07-29_m20_sdk_gazebo_locomotion.md](docs/devlog/2026-07-29_m20_sdk_gazebo_locomotion.md)，
对应 6A-1 测试见
[docs/test_reports/2026-07-29_m20_sdk_gazebo_locomotion.md](docs/test_reports/2026-07-29_m20_sdk_gazebo_locomotion.md)。
原 Gazebo 6A-2 路线已由直接 MuJoCo 完整联仿取代。MuJoCo 后端、同源碰撞世界、
SDK ready/fault 门控和新的完整启动入口已实现；无 GUI 验收已覆盖稳定站立、实时
点云、低速速度闭环、原版 SCAN 近距离目标，以及历史 0.70 m 场景的 11 步完整任务。
完整任务通过受支持的当前步骤重试完成，最终回起点误差 `0.0855 m`；因此已把 MuJoCo
单步默认超时从平面模式的 `180 s` 独立调整为 `300 s`。新的超时配置仍需一次无重试
复验，当前 0.90 m 默认资产也需完整动态复验；重复转向台架和实机等价性不由历史
结果替代。
实现记录见
[docs/devlog/2026-07-29_mujoco_full_cosimulation.md](docs/devlog/2026-07-29_mujoco_full_cosimulation.md)，
当前放行范围和实测数据见
[docs/test_reports/2026-07-29_mujoco_full_cosimulation.md](docs/test_reports/2026-07-29_mujoco_full_cosimulation.md)。
自由导航直线、90°、180°六目标的轮腿协同基线与原因分析见
[docs/devlog/2026-07-29_navigation_motion_baseline.md](docs/devlog/2026-07-29_navigation_motion_baseline.md)，
对应量化结果和原始数据索引见
[docs/test_reports/2026-07-29_navigation_motion_baseline.md](docs/test_reports/2026-07-29_navigation_motion_baseline.md)。
已按该基线加入自动导航滚动适配，SCAN规划与官方ONNX均未修改；实现与参数说明见
[docs/devlog/2026-07-29_rolling_navigation_adapter.md](docs/devlog/2026-07-29_rolling_navigation_adapter.md)，
同组六目标A/B结果见
[docs/test_reports/2026-07-29_rolling_navigation_adapter.md](docs/test_reports/2026-07-29_rolling_navigation_adapter.md)。
绕障停车后轨迹脱节修复、统一 candidate 链路和双圆保护实现见
[docs/devlog/2026-07-30_candidate_safety_freeze.md](docs/devlog/2026-07-30_candidate_safety_freeze.md)，
RViz/MuJoCo 同目标回归见
[docs/test_reports/2026-07-30_candidate_safety_freeze.md](docs/test_reports/2026-07-30_candidate_safety_freeze.md)。
有界机体系速度闭环的实现、坐标契约和安全降级见
[docs/devlog/2026-07-31_m20_velocity_feedback.md](docs/devlog/2026-07-31_m20_velocity_feedback.md)，
首次同任务 MuJoCo A/B 数据与暂不提升默认值的判定见
[docs/test_reports/2026-07-31_m20_velocity_feedback_ab.md](docs/test_reports/2026-07-31_m20_velocity_feedback_ab.md)。
默认双区域 11 步完整动力学任务与段间停稳结果见
[docs/test_reports/2026-08-02_multifloor_mujoco_11_step.md](docs/test_reports/2026-08-02_multifloor_mujoco_11_step.md)。
针对上边界和共享原点的 6 次独立冷启动恢复复验见
[docs/test_reports/2026-08-02_boundary_recovery_cold_starts.md](docs/test_reports/2026-08-02_boundary_recovery_cold_starts.md)，
实现边界见
[docs/devlog/2026-08-02_boundary_recovery_validation.md](docs/devlog/2026-08-02_boundary_recovery_validation.md)。

## 配置入口

系统级配置位于：

```text
config/flat_multifloor_system.yaml
config/dense_four_corner_system.yaml
config/route_challenge_system.yaml
config/rviz_v1_baseline.yaml
```

前两个是冻结的稳定配置，第三个是路线受扰动实验配置，第四个锁定稳定配置、PCD
和官方 M20 URDF 的 SHA-256。系统 YAML 不是 ROS 2 参数文件；配置加载器读取、校验后
再把各模块所需参数传给 ROS 节点，避免把楼层、地图、电梯和任务信息散落在多个
launch 文件中。

## 构建与运行

### Foxy x86 实机入口（接口适配完成，尚未台架放行）

实机目标是背部 x86/Ubuntu 20.04/Foxy 单机部署。入口默认组合双 RoboSense + `/IMU`
的 Elevator-LIO、LIO/出厂速度融合适配器、原 SCAN/任务/安全链，以及可二选一的官方
`basic_server`/direct ROS 运动后端；默认使用 `basic_server` 且禁止取得运动权限：

```bash
source /opt/ros/foxy/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI

ros2 launch m20_warehouse_inspection \
  inspection_mission_hardware.launch.py \
  system_config:=/ABSOLUTE/PATH/TO/REAL_SITE.yaml
```

direct ROS 改用以下参数，其他链路不变：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_hardware.launch.py \
  factory_transport:=direct_ros \
  system_config:=/ABSOLUTE/PATH/TO/REAL_SITE.yaml
```

完整指南已补齐并逐字段校验 `NavCmd/MotionInfo/MotionState/Gait`；实际放行前仍需与目标
机器镜像执行 `ros2 interface show` 和 QoS 对照。完整链路、Foxy 差异、Elevator-LIO
接口和待补信息见
[docs/real_robot_migration/03_foxy_x86_elevator_lio_integration.md](docs/real_robot_migration/03_foxy_x86_elevator_lio_integration.md)。
从单层 MuJoCo 验收、源码上传、Foxy 原生构建到台架/空场/F1 单层放行的现场操作顺序见
[docs/real_robot_migration/04_m20_pao_step_by_step_deployment.md](docs/real_robot_migration/04_m20_pao_step_by_step_deployment.md)。
此入口不能使用仿真地图作为现场地图，也不代表已经完成实机运动放行。

### 完整 MuJoCo 动力学联仿（当前推荐）

第一个终端启动一次完整系统。默认使用 0.90 m 高密度双场景，RViz 中仍显示
原版 SCAN 的实时点云、占据、膨胀和轨迹；运动执行改为官方 ONNX 策略与 MuJoCo
16 关节动力学。主入口默认使用 `execution_profile:=scan_native`：保持原版 SCAN
搜索、B-spline、控制器和可视化，仅叠加 M20 机体几何参数（硬规划双圆半径
`0.28 m`、优化器软距离 `0.17 m`、圆心偏置 `0.18 m`），并保留原控制器的全向
Twist，不启动项目后来增加的滚动适配、足迹预测、恢复状态机或外部轨迹时钟
hold。需要精确复现上游几何时可显式传入
`clearance_config:=.../clearance_vendor.yaml`：

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install --user "mujoco==3.10.0"
source /home/virdyn/robodog_nav_system/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=79
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI=file:///home/virdyn/robodog_nav_system/install/\
m20_warehouse_inspection/share/m20_warehouse_inspection/config/cyclonedds_local.xml

ros2 launch m20_warehouse_inspection \
  inspection_mission_mujoco.launch.py
```

启动后先等待日志出现：

```text
MuJoCo M20 backend ready: official SDK stand-up is stable
```

随后可直接用 RViz 的 `2D Goal Pose` 自由导航。自动巡检仍只需第二个终端：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=79
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI=file:///home/virdyn/robodog_nav_system/install/\
m20_warehouse_inspection/share/m20_warehouse_inspection/config/cyclonedds_local.xml

ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id dense_four_corner_patrol
```

MuJoCo 原生三维窗口现在默认打开。如需无界面运行或降低重复渲染负载，启动时传入
`use_mujoco_viewer:=false`。窗口默认使用以 `base_link` 为中心、4 m 距离的自动跟随
相机；例如 `mujoco_viewer_distance:=6.0` 可扩大周边视野。路线受扰动 profile 使用
`route_challenge_mission_mujoco.launch.py`。完整 MuJoCo 入口的每步导航超时默认是
`300 s`，需要实验覆盖时可传入 `navigation_timeout_sec:=...`。纯 RViz 平面后端没有
删除，下面的旧入口继续用于 SCAN/任务回归和与历史结果对比。

官方 ONNX 同时输出 12 个腿关节位置目标和 4 个轮关节速度目标。它只接收前向、横向、
偏航三个连续速度量，没有“轮式/四足”离散切换输入；因此腿部在站立、直行和转弯中
始终参与姿态平衡。系统的 `WHEEL_CRUISE`、`COORDINATED_TURN` 和
`LATERAL_MANEUVER` 是限速/诊断意图，不是假步态开关。停车意图下，MuJoCo 后端保留
官方腿部平衡输出，只对四个轮端施加零速阻尼制动；开始运动时自动释放。

默认速度链是
`closed_loop_controller -> cmd_vel_raw -> floor/e-stop gate -> cmd_vel_safe -> backend`。
这个门只负责多层切换、任务保持、急停、地图/里程计就绪和命令超时；在
`scan_native` 下不判定障碍，也不修改 SCAN 轨迹。MuJoCo 后面仍保留后端就绪门，
官方 SDK 最终按照其自身能力把速度限制为 `0.45/0.20/0.65`，这是当前 M20 替换
相对原理想 Go2 运动学模型保留的明确差异，后续单独做轮足运动适配分析。

历史 M20 适配链没有删除。如需 A/B 对照可显式追加
`execution_profile:=m20_safe`，此时才启动 `m20_navigation_adapter`、
`m20_collision_guard`、有界恢复和外部轨迹 hold。真机入口目前固定使用
`m20_safe`，在完成空场与障碍实测前不能改为原生直通。

步骤一，只启动一次完整系统：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to m20_warehouse_inspection
source install/setup.bash

ros2 run m20_warehouse_inspection m20_validate_config --require-assets
ros2 launch m20_warehouse_inspection inspection_mission_rviz.launch.py
```

这是唯一的日常系统启动入口。启动后不自动执行任务：

- 在 RViz 使用 `2D Goal Pose`，目标直接发送到 SCAN，机器狗像原项目一样自由导航；
- 需要自动巡检时保持第一个终端和 RViz 不动，执行步骤二。

步骤二，在第二个终端启动自动巡检：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
ros2 run m20_warehouse_inspection m20_start_inspection
```

该命令只触发第一个终端中已经等待的 mission executor，并持续打印任务反馈；不会再
启动第二套地图、SCAN 或任务节点。任务会自动导航两个 F1 巡检点、进入 E1、切换到
F2、依次巡检 F2_A 与 F2_B，再经不驻留的 `F2_RETURN_VIA_A` 返回共享原点
`F2_TERMINAL`。RViz 中的文字
Marker 会显示当前任务步骤、楼层和 generation。

### 高密度四角巡检副本

该副本具有独立配置、地图目录、任务 ID 和启动入口，不会覆盖上面的基线场景。

步骤一：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash

ros2 launch m20_warehouse_inspection \
  dense_four_corner_mission_rviz.launch.py
```

步骤二：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash

ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id dense_four_corner_patrol
```

路线共有 11 步：

```text
F1 左下 -> F1 右下 -> F1 右上 -> F1 左上
-> 共享原点切换至 F2
-> F2 左下 -> F2 右下 -> F2 右上 -> F2 左上
-> 共享原点切回 F1 -> 全流程起点 (-37, 0)
```

两次楼层切换都要求机器狗先自主到达共享原点并停车，只切换 active PCD，
不修改机器狗世界位姿。两个场景的障碍物数量均为 246，其中 6 个 1 m × 1 m
固定障碍物交错布置在下、右、上三段巡检过道边缘，另外 240 个由固定种子生成。
实际生成资产的最近独立障碍物边界距离为 `0.900236 m`。当前生成器不创建连体
障碍物组，因此所有随机和固定障碍物都按不小于 0.9 m 的规则检查；以后若加入
连体组，需要在元数据中显式标识后再放宽组内间距。

注意：这里的0.90 m是“两个独立障碍物本体之间”的生成约束，不等同于机器人和
单个障碍物之间的规划净空。当前资产已通过0.30 m保护半径的膨胀连通性回归，
默认资产的完整MuJoCo 11步动态任务已完成11/11；这仍不代替新地图或实机部署时的
独立净空验收。

### 路线受扰动测试场景

该场景复制高密度四角任务，不修改冻结地图。12 个不同尺寸和朝向的固定障碍物分别
进入左侧下行、底边、右边、顶边、切换斜向通道和最终返航通道，要求 SCAN 偏离名义
直线绕行。

步骤一：

```bash
ros2 launch m20_warehouse_inspection \
  route_challenge_mission_rviz.launch.py
```

步骤二：

```bash
ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id route_challenge_four_corner_patrol
```

每场景共 252 个障碍物（240 随机 + 12 固定），实际最小边界间距为
`0.702172 m`。静态验收确认 7 类任务直达航段都与固定障碍物相交，并且占据图按
`0.30 m` 安全轮廓膨胀后所有任务点仍连通。该 profile 当前用于后续运行测试，
不继承冻结基线已有的 10/10 多轮结论。

### 障碍物间距对比测试

`spacing_sweep` 在相同双场景尺寸、252 个障碍物、随机种子、固定障碍物和11步
巡检任务下提供 `0.70 / 0.80 / 0.90 / 1.00 m` 四个独立档位。原
`route_challenge` 地图不会被覆盖。

步骤一选择一个档位：

```bash
ros2 launch m20_warehouse_inspection \
  spacing_sweep_mission_rviz.launch.py spacing:=0.70
```

`spacing` 只能取 `0.70`、`0.80`、`0.90` 或 `1.00`。步骤二在另一个终端启动
相同的巡检任务：

```bash
ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id spacing_sweep_four_corner_patrol
```

四套已交付地图的实际最近障碍物本体间距分别是：

| 档位 | 实际最小间距 | `0.30 m` 膨胀后任务航段连通 |
|---:|---:|---:|
| `0.70 m` | `0.702172 m` | `11/11` |
| `0.80 m` | `0.805815 m` | `11/11` |
| `0.90 m` | `0.900236 m` | `11/11` |
| `1.00 m` | `1.001421 m` | `11/11` |

首段 `(-37, 0) -> (-36, -16)` 原生SCAN无界面动态单次对比均成功，耗时依次为
`32.58 / 29.32 / 28.52 / 30.24 s`。这是首段筛选结果，不等同于四套完整11步
任务的多轮稳定性结论。详细条件和日志统计见
[`docs/test_reports/2026-07-28_obstacle_spacing_sweep.md`](docs/test_reports/2026-07-28_obstacle_spacing_sweep.md)。

需要确定性重新生成四套资产时运行：

```bash
python3 src/m20_warehouse_system/bringup/m20_warehouse_inspection/tools/generate_spacing_sweep.py
```

### 独立工作空间复现

从项目源码根目录准备一个不存在或为空的目标：

```bash
src/m20_warehouse_system/bringup/m20_warehouse_inspection/tools/prepare_isolated_workspace.sh \
  /path/to/m20_warehouse_ws
```

脚本复制十个项目包，并导入锁定 revision 的 SCAN-Planner、官方模型和 M20 SDK；
SDK 的 ROS `cmd_vel` 适配由 SHA-256 锁定补丁恢复，不复用当前工作空间的
`build/`、`install/`。完整的在线/本地镜像、MuJoCo Python 依赖、严格 overlay 清理
和 22 包构建步骤见
[docs/isolated_workspace.md](docs/isolated_workspace.md)。

仅运行完整节点图、不启动 RViz：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_rviz.launch.py use_rviz:=false
```

自由导航单目标冒烟测试可在无界面启动后运行：

```bash
ros2 run m20_warehouse_inspection m20_navigation_motion_probe \
  --output-directory /tmp/m20_navigation_smoke \
  --target straight_2p5m -34.5 0.0 \
  --goal-timeout-sec 60
```

本机若出现默认 DDS participant 数量不足，可使用项目内的 CycloneDDS 容量配置。
该配置不强制网卡，避免 WSL/Hyper-V 环境的 `lo` 静默丢弃 PointCloud2：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=79
export CYCLONEDDS_URI=file://$(ros2 pkg prefix m20_warehouse_inspection)/share/m20_warehouse_inspection/config/cyclonedds_local.xml
```

`ROS_DOMAIN_ID` 可换成未占用的 0–232 数值，但启动系统和巡检脚本的两个终端必须
使用同一个值。真实多机部署应按现场 DDS 网络方案重新选择接口和 domain。

阶段 5 的无 GUI 系统回归入口：

```bash
# 20 次 F1/F2 交替切图
ros2 launch m20_warehouse_inspection phase5_regression.launch.py \
  mode:=switch_stress switch_iterations:=20 use_rviz:=false

# 10 次当前配置完整任务
ros2 launch m20_warehouse_inspection phase5_regression.launch.py \
  mode:=mission_stress mission_runs:=10 use_rviz:=false

# 非法请求、旧代次、取消、停止和故障重试
ros2 launch m20_warehouse_inspection phase5_regression.launch.py \
  mode:=fault_injection use_rviz:=false
```

回归进程结束时该专用 launch 会有序关闭完整节点图。自动化调用方应同时检查进程结果
和 `PHASE5_*_PASS` 日志标记。

`flat_multifloor_rviz.launch.py`、`f1_scan_rviz.launch.py` 和
`multifloor_scan_rviz.launch.py` 仅供分阶段开发回归，不属于操作者日常启动步骤。

### 任务控制

活动任务使用 `/m20/mission/control` 控制。`command` 常量为
`PAUSE=1`、`RESUME=2`、`STOP=3`、`RETRY_CURRENT=4`：

```bash
ros2 service call /m20/mission/control \
  m20_warehouse_interfaces/srv/ControlMission \
  "{mission_id: two_floor_warehouse_demo, command: 1}"

ros2 service call /m20/mission/control \
  m20_warehouse_interfaces/srv/ControlMission \
  "{mission_id: two_floor_warehouse_demo, command: 2}"
```

暂停会取消当前子导航、清理路线并保持零速；恢复会基于当前位置重新规划。原子楼层切换
期间拒绝 pause，避免把地图事务停在不可判定的中间状态。`STOP` 和未恢复故障会保留
安全 hold，必须由上层明确处理。

单个 typed 导航目标也可通过 `/m20/navigation/navigate` 发送；目标必须显式携带当前
`floor_id` 和 `map_generation`。RViz 的 `2D Goal Pose` 兼容入口仍保留用于人工调试，
任务层不使用该无类型入口判断导航结果。

### 楼层切换

正常流程先导航到电梯 cabin 触发点，再发送 Action：

```bash
ros2 action send_goal --feedback /m20/floor_switch \
  m20_warehouse_interfaces/action/SwitchFloor \
  "{source_floor: F1, target_floor: F2, elevator_id: E1}"
```

反向切换使用：

```bash
ros2 action send_goal --feedback /m20/floor_switch \
  m20_warehouse_interfaces/action/SwitchFloor \
  "{source_floor: F2, target_floor: F1, elevator_id: E1}"
```

Action 会校验机器人是否位于共享原点的触发容差内。默认 connected profile 不创建
`/m20/sim/set_pose` 客户端；切图提交后还会再次核对 odom 仍位于共享门洞，防止用地图
切换掩盖位姿跳变。

需要根据配置重新生成地图时：

```bash
ros2 run m20_warehouse_inspection m20_generate_maps
```

高密度副本使用：

```bash
ros2 run m20_warehouse_inspection m20_generate_maps \
  --config "$(ros2 pkg prefix m20_warehouse_inspection)/share/\
m20_warehouse_inspection/config/dense_four_corner_system.yaml" \
  --package-root "$(ros2 pkg prefix m20_warehouse_inspection)/share/\
m20_warehouse_inspection"
```

相同配置和 seed 必须生成相同哈希。声明 `replica_of` 的楼层必须与源楼层具有完全
相同的内部障碍和内部 PCD；若两层在不同相邻边开门，完整文件哈希应不同。
生成器也可以在首次 colcon 构建之前直接从源码运行，不要求 ROS 包索引可用。

## 当前关键接口

```text
/m20/visualization/all_floors_cloud  F1+F2，总览专用
/m20/visualization/inactive_floors_cloud 默认 RViz 的非活动楼层点云
/m20/map/active_global_cloud         仅当前 active floor
/m20/map/state                       typed floor/generation/ready/phase
/m20/map/switch                      带 expected generation 的 CAS 服务
/m20/visualization/floor_markers     区域边界、楼层名称和 active 状态
/map_generator/global_cloud          原版 RViz/renderer 的 active PCD
/quad_0/cloud                        原版 SCAN 规划输入和实时点云显示
/quad_0/sensor_cloud                 原版 renderer 调试点云
/m20/sensing/state                   新鲜帧计数、点数和 XY 边界
/move_base_simple/goal               默认 RViz 与自动任务共用的 SCAN 目标
/m20/navigation/navigate             floor/generation-aware 导航 Action
/m20/navigation/state                typed 导航状态
/m20/navigation/reset                清空原生 SCAN/GridMap/目标/轨迹状态
/grid_map/occupancy                  SCAN 在线三维占据点云（非 PGM/OccupancyGrid）
/m20/navigation/cmd_vel_raw          SCAN 原始速度
/planning/tracking_direction         M20 B-spline 前进/倒车执行方向诊断
/m20/navigation/cmd_vel_candidate    仅 m20_safe：滚动化后的预安全候选速度
/m20/navigation/candidate_mode       仅 m20_safe：预安全滚动/转向意图
/m20/navigation/velocity_feedback_state 仅 m20_safe：速度闭环状态、误差和补偿
/m20/mission/run                     自动巡检任务 Action
/m20/mission/control                 pause/resume/stop/retry 服务
/m20/mission/state                   typed 任务状态
/m20/control/mission_hold            任务级安全保持
/m20/visualization/mission_marker    RViz 任务文字状态
/m20/control/collision_stop          仅 m20_safe：前视碰撞保护输出
/m20/control/collision_guard_diagnostic 仅 m20_safe：首个双圆碰撞样本诊断
/m20/control/collision_recovery_available 仅 m20_safe：有界滚动恢复许可
/m20/control/collision_recovery_cmd  仅 m20_safe：短时滚动恢复指令
/m20/control/execution_hold          仅 m20_safe：SCAN 执行时钟保持
/m20/control/cmd_vel_safe            所有后端唯一允许订阅的速度
/m20/sim/body_pose                   当前选定后端位姿；完整联仿由 MuJoCo 发布
/m20/sim/backend_ready               物理加载、SDK 站立和姿态稳定门控
/m20/sim/backend_fault               跌倒、过度倾斜或非有限状态故障
/m20/sim/dynamics_state              实时率、接触、姿态、力矩和命令时效诊断
/m20/locomotion/cmd_vel_sdk          安全限幅后送入官方 SDK 的速度
/JOINTS_CMD                          官方 SDK 输出的 16 关节 PD/前馈命令
/JOINTS_DATA                         MuJoCo 回送的官方坐标关节状态
/IMU_DATA                            MuJoCo 回送的官方 IMU 状态
/m20/sim/set_pose                    旧分离区域测试 profile 的可选位姿服务
/m20/floor_switch                    双向楼层切换 Action
/m20/floor_switch/state              当前事务阶段（锁存）
```

地图和状态快照使用 reliable、transient-local、depth=1。高频点云由第三方
`pcl_render_node` 按原场景的 360 度激光参数产生，SCAN 与 RViz 均订阅
`/quad_0/cloud`。总览点云禁止接入 SCAN-Planner。

### 地图颜色与围栏语义

默认 RViz 直接使用 SCAN 首场景的显示树。active floor 的 Global Map 使用原项目
`AxisColor + Flat Squares`，Sensor Cloud、Occupancy、Inflated Occupancy、滑窗、
Goal、轨迹和 TF 均恢复原话题与显示属性。非活动楼层通过
`Copied Scene (Inactive Floor)` 以固定蓝灰色显示；地图服务器保证 active floor 不会
同时进入该图层，因此移动相机不会再出现两个 PCD 图层争抢深度导致的黄/粉色切换。
`/m20/visualization/all_floors_cloud` 保留给双区域总览分析工具。
`SCAN Occupancy` 和 `SCAN Inflated Occupancy` 均恢复原项目的
`AxisColor + Squares` 属性；后者仍表示随机器人移动的安全膨胀层，可在 Displays
中单独关闭做对照。

区域边框的 RViz `Marker` 本身只负责状态显示，不参与碰撞计算。真正的围栏同时存在于：

- active PCD 的四周 2 m 高墙面点，进入 SCAN 局部感知和占据/膨胀窗口；
- active PCD 的墙面被 SCAN 在线 GridMap 占据化，并同时进入独立点云
  `collision_guard` 的内存栅格。

完整系统使用前后双圆，圆心沿机体航向 `±0.18 m`。SCAN 与原项目一致，采用
`0.25 m` 硬规划半径和 `0.20 m` 优化器软距离；rebound A* 仍是原版二值碰撞
代价，B-spline 也没有额外全局净空项。独立保护器对 SCAN 的在线占据点云构建
机器人中心内存栅格，按 `0.28 m` 机身半径加 `0.02 m` 余量检查候选命令，所以
规划层与执行层保持分层：前者保持原版算法，后者负责 M20 实际执行安全。规划与保护只消费当前
active floor，因此非活动区域的围栏不会错误影响当前楼层。F1 右围栏和 F2 左围栏只在
`y=[-2,2]` 留门，其余边界仍参与规划与碰撞保护。机器狗先从 F1 运动到门洞中心
`(0,0)`，停车并原地切换 active map 后，才继续执行 F2 任务。碰撞保护允许共享边缘
0.35 m 的定位容差，但仍读取边界占用单元，所以没有门洞的围栏不会被放开。
正式 `conservative` 档的恒指令前视为0.70 s：最高0.45 m/s时前视0.315 m，
约为MuJoCo实测最坏停车位移0.156 m的2倍。足迹和0.05 m余量没有缩小。

## 窄通道净空口径

障碍物之间的最小间距不等于机器狗可稳定通过宽度。当前系统将机身、SCAN优化、
独立碰撞保护和占据图分辨率分别计算：

- M20确认机体长宽为`0.82 × 0.51 m`；
- 上游`0.25/0.18 m`双圆轴向外廓约为`0.86 × 0.50 m`，宽度小于M20约1 cm；
- 完整系统默认`m20`档：硬半径`0.28 m`、偏置`0.18 m`，轴向外廓约
  `0.92 × 0.56 m`，长宽方向分别留下总计约0.10/0.05 m余量；
- SCAN优化器软距离同步调整为`0.17 m`，硬半径加软距离仍为0.45 m；
- `m20_safe`独立保护有效半径为`0.28 + 0.02 = 0.30 m`；默认
  `scan_native`不启动该保护器；
- SCAN GridMap分辨率为0.05 m，正式场景障碍物表面采样间隔为0.10 m；
- 正式系统当前首个正常候选宽度为0.90 m，0.80 m及以下不能直接作为现场放行值。

高度方向保持原生SCAN配置：当前RViz/Action目标的Z值来自首帧机身中心高度，
`grid_map.body_height`只影响未启用的`initial_path`支撑面输入；上下Z膨胀仍为
`0.10/0.40 m`。因此本轮只完成地面仓库的平面净空适配，不能据此放行可钻越的横梁、
桌面或货架底部，真机需另测垂直外廓和雷达盲区。

双圆柱是兼顾可通行性的方向相关近似，不是M20长方体的严格外接轮廓。若要求双圆
完整覆盖`0.82 × 0.51 m`矩形四角，优化后的半径也需约0.327 m，并会在0.05 m
GridMap上接近此前已否决的0.35 m离散膨胀，封闭0.90 m通道。因此当前参数必须配合
MuJoCo实体接触统计和后续真机净空验收，不能仅凭参数宣称绝对安全。

项目另带0.05 m分辨率的0.60～0.90 m标准门洞系列。动态MuJoCo结果为：
0.75 m被预测保护安全拒绝；0.80 m单次虽通过，但连续保护余量只有2.3 mm；
0.90 m三次冷启动和调参后回归全部通过；加入航向对正后的一次跟踪回归把最小连续
保护余量从约52.3 mm提高到75.9 mm，且无预测/当前停车或实体接触。因此正式最小
间隔继续为0.90 m。

参数报告：

```bash
ros2 run m20_scan_navigation m20_clearance_report \
  --profile m20 --map-resolution 0.05
```

完整测试入口：

```bash
ros2 launch m20_warehouse_inspection \
  narrow_passage_mission_mujoco.launch.py \
  width:=0.90 clearance_profile:=m20
```

单障碍原生绕行入口：

```bash
ros2 launch m20_warehouse_inspection \
  clearance_maneuver_mission_mujoco.launch.py \
  system_config:=$(ros2 pkg prefix m20_warehouse_inspection)/share/\
m20_warehouse_inspection/config/clearance_direct_system.yaml

ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id single_obstacle_direct_bypass
```

详细推导见
[静态净空记录](docs/devlog/2026-07-29_narrow_passage_clearance.md)和
[动态整定记录](docs/devlog/2026-07-30_dynamic_clearance_tuning.md)。
本轮航向跟随、旋转恢复和否决实验见
[航向控制记录](docs/devlog/2026-07-30_heading_alignment_control.md)与
[对应测试报告](docs/test_reports/2026-07-30_heading_alignment_control.md)。
上述 SCAN 内部航向和净空试验现已撤回；当前主线恢复固定第三方 SCAN 参数与
原闭环控制行为，仅保留话题、Action、切层和 execution hold 接口。恢复范围见
[SCAN 原项目基线恢复记录](docs/devlog/2026-07-31_scan_vendor_baseline_restore.md)，
构建与一致性结果见
[对应测试报告](docs/test_reports/2026-07-31_scan_vendor_baseline_restore.md)。
原版场景 1 与当前 M20 的障碍间距、理想全向执行器、官方轮足动力学和同路线
实测对照见
[执行链对照记录](docs/devlog/2026-07-31_scan_m20_execution_comparison.md)及
[测试报告](docs/test_reports/2026-07-31_scan_m20_execution_comparison.md)。
该对照确认当前首要问题位于 M20 运动适配/动态执行链，不再通过修改 SCAN
规划参数补偿。
Go2 官方 Sport 接口、原 SCAN 理想仿真、M20 公开 RL 部署 SDK 与两台机器人
模型/转向约束的逐层对照见
[Go2/M20 SDK 与模型对照](docs/devlog/2026-07-31_go2_m20_sdk_model_comparison.md)。
该记录明确：原 SCAN 仓库不包含 Unitree SDK 适配器，M20 的轮腿参与由 ONNX
连续策略共同决定，后续差异统一收敛到平台能力与运动 backend，而不进入规划器。

该平台边界现已实现为版本化 `m20_policy_v1` 能力配置：导航适配、碰撞预测、最终
SDK 门和官方策略桥共享同一组 M20 包线；碰撞恢复增加时间、位移和无进展上限。
实现与参数推导见
[M20 平台能力配置与有界恢复记录](docs/devlog/2026-07-31_m20_capability_profile_bounded_recovery.md)。
官方 SDK + MuJoCo 单障碍和六目标最终动态结果见
[双向轨迹跟踪与有界恢复报告](docs/test_reports/2026-07-31_m20_bidirectional_tracking_bounded_recovery.md)：
六目标 `6/6`，两个 180° 回程的横向误差分别为 `0.0084 m / 0.0051 m`，无恢复、
预算耗尽或实体障碍接触。

后续冷启动已把倒车判定收敛为“新 B-spline 多点直线校验 + 后轴对齐 + 倒车航向
锁存”，避免曲线误判和末端切向抖动。新的六目标仍为 `6/6`。实测速度反馈在开阔
路线改善用时与终点误差，但单障碍重复样本的最小硬余量仍有较大波动，因此主配置
继续默认关闭反馈。实现过程见
[倒车与速度反馈记录](docs/devlog/2026-08-01_m20_reverse_alignment_velocity_feedback_ab.md)，
量化结果见
[再资格验证报告](docs/test_reports/2026-08-01_m20_velocity_feedback_requalification.md)。

### M20 SDK 隔离速度试验

可在不启动 SCAN、地图切换、碰撞保护和任务执行器的情况下，直接测试官方策略与
M20 MuJoCo 模型：

```bash
ros2 launch m20_warehouse_inspection \
  m20_motion_envelope_mujoco.launch.py \
  use_viewer:=false suite:=smoke \
  output_directory:=/tmp/m20_motion_envelope
```

`suite` 支持 `smoke`、`axial`、`navigation`、`turn_matrix`、`reverse` 和
`full`。精确冷启动
单格可用 `case_name:=turn_005_020`；正式速度边界必须以单格重复结果为准，不把同一
进程内不稳定动作后的结果直接当成独立样本。默认测试仍使用生产接口限幅
`0.45 / 0.20 / 0.65`，不会改变 SCAN 参数或主系统默认启动方式。设计、源码核查和
首轮结果见
[M20 SDK 速度包线第一级记录](docs/devlog/2026-07-31_m20_command_envelope_phase1.md)。

## 独立工作空间边界

已验证可迁入独立工作空间的项目包边界是：

```text
src/m20_warehouse_system/bringup/m20_warehouse_inspection
src/m20_warehouse_system/common/m20_warehouse_interfaces
src/m20_warehouse_system/common/m20_official_description
src/m20_warehouse_system/navigation/m20_multifloor_map
src/m20_warehouse_system/navigation/m20_scan_planner
src/m20_warehouse_system/navigation/m20_scan_navigation
src/m20_warehouse_system/motion/m20_locomotion_control
src/m20_warehouse_system/simulation/m20_mujoco_backend
src/m20_warehouse_system/safety_mission/m20_inspection_core
src/m20_warehouse_system/simulation/m20_warehouse_sim
src/drdds
dependencies.repos 中固定版本的 SCAN 算法依赖
dependencies_sdk.repos 中固定版本的 M20 SDK（其旧 `drdds` 副本保持隔离）
```

MuJoCo 与 M20 SDK 已成为完整动力学 profile 的受控依赖，但纯 RViz 冻结基线仍可
单独使用平面后端。Gazebo 仅保留为非主线实验入口。`/m20/sim/set_pose` 只供旧
仿真/测试 profile 使用；MuJoCo connected profile 不使用它。实机迁移时替换物理、
定位和点云后端，任务 Action、安全链路及 floor/generation 契约保持不变。
