# 实物部署准备 01：现有完整仿真系统模块与职责清单

> 状态：实物迁移前基线说明  
> 基线日期：2026-08-10  
> 当前用户入口：`m20_warehouse_inspection`  
> 当前构建闭包：10 个项目模块包 + 12 个锁定第三方包，共 22 个 ROS 2 包

## 1. 文档目的

本文先冻结“当前完整仿真系统由什么组成、每个目录负责什么、实机阶段应如何处理”的
共同认知，作为后续实物部署设计和改造的起点。

这里的“整合为一个功能包”是指：

- 操作层只通过 `m20_warehouse_inspection` 配置、构建和启动完整系统；
- 实现层继续保留消息、地图事务、导航、安全、运控、模型和第三方库之间的 ROS 2
  包边界；
- 不把不同许可证的第三方源码、接口生成包和业务代码物理拼成一个无法独立测试的
  CMake 包；
- 迁移到实机时，用真实传感器、定位和硬件执行后端替换仿真数据源，任务与导航上层
  尽量保持不变。

当前闭包已经完成全新构建、包级测试及 RViz 平面自由导航冒烟测试。验证记录见：

- [`../package_inventory.md`](../package_inventory.md)
- [`../test_reports/2026-08-10_integrated_navigation_smoke.md`](../test_reports/2026-08-10_integrated_navigation_smoke.md)

官方运动层的详细对照、已实现后端和真机前置条件见：

- [`02_official_motion_layer_analysis_and_backend_switch.md`](02_official_motion_layer_analysis_and_backend_switch.md)

## 2. 完整仿真总体链路

### 2.1 导航与运动命令链

```text
RViz 点击目标 / 自动巡检任务
  -> /move_base_simple/goal 或 /m20/navigation/navigate
  -> SCAN GridMap + 路径搜索 + B-spline 重规划
  -> m20_scan_planner/closed_loop_controller
  -> /m20/navigation/cmd_vel_raw
  -> m20_locomotion_control/m20_navigation_adapter
  -> /m20/navigation/cmd_vel_candidate
  -> m20_inspection_core/collision_guard + safety_supervisor
  -> /m20/control/cmd_vel_safe
  -> 二选一执行后端：
       A. m20_warehouse_sim：RViz 平面运动学
       B. m20_locomotion_manager -> M20 SDK/ONNX -> /JOINTS_CMD
          -> m20_mujoco_backend：MuJoCo 16 执行器动力学
```

这里有一个重要边界：SCAN 只负责规划和生成原始机体系速度；M20 滚动适配、碰撞否决、
安全停车和 SDK 限幅都位于其下游。实机部署不应绕过这些安全层，也不应让多个节点同时
向机器狗底层发送最终命令。

### 2.2 地图、感知与状态反馈链

```text
系统 YAML + 当前楼层 PCD/metadata
  -> m20_warehouse_inspection/m20_flat_map_server
  -> active floor + generation + active global PCD cloud
  -> 仿真 local sensing 或后续真实激光雷达
  -> /quad_0/cloud、/quad_0/sensor_cloud、/quad_0/lidar_pose
  -> SCAN GridMap

RViz 运动学后端或 MuJoCo 后端
  -> /m20/sim/body_pose + TF + joint_states
  -> local sensing、导航网关、碰撞保护、速度适配
```

`/m20/sim/body_pose` 是目前各模块共同使用的位姿/速度契约名称。实机迁移时可以先通过
适配节点继续发布这个兼容话题，降低一次性修改风险；系统稳定后再统一改成正式
`/odom`、`map -> odom -> base_link` 接口。

### 2.3 多层任务链

```text
/m20/mission/run
  -> m20_mission_executor
  -> 当前楼层巡检点 /m20/navigation/navigate
  -> 到达 connector 触发点并连续停稳
  -> /m20/floor_switch 原子事务
  -> transport adapter（当前 timed_hold；实机可换 external_action）
  -> active floor/generation 提交
  -> 新地图、新定位和新鲜点云确认
  -> 释放 hold，继续下一楼层任务
```

地图切换、运输执行和位姿交接已经参数化分离。当前平面双区域只是在共享原点停车后切换
活动 PCD，不传送机器人；实机可将 transport adapter 替换成电梯或升降平台 provider，
无需重写任务执行器。

## 3. 十个项目模块包

### 3.1 `m20_warehouse_inspection`：唯一系统入口与资产所有者

- 目录：`src/m20_warehouse_system/bringup/m20_warehouse_inspection/`
- 负责：总 launch、系统 YAML、地图/metadata、RViz 配置、配置校验、地图生成、验收
  探针、测试报告和开发记录。
- 主要运行入口：
  - `f1_scan_rviz.launch.py`：单场景原生 SCAN 自由导航；
  - `inspection_mission_rviz.launch.py`：RViz 平面后端完整巡检；
  - `inspection_mission_mujoco.launch.py`：SCAN + M20 SDK/ONNX + MuJoCo 完整动力学联仿；
  - `m20_start_inspection`：从第二个终端启动配置中的自动巡检任务。
- 主要工具：`m20_flat_map_server`、`m20_generate_maps`、`m20_validate_config` 和阶段验收/
  运动探针。
- 实机处理：**保留并成为实机总入口**。新增实机 profile 和 launch；仿真 launch、
  地图生成器及测试探针继续保留，但不能进入生产启动图。

关键子目录：

| 子目录 | 内容 |
| --- | --- |
| `config/` | 双区域、任务、切层、导航和测试 profile |
| `launch/` | 完整系统组合入口 |
| `maps/` | PCD 和 JSON metadata 资产；不含 PGM/YAML |
| `rviz/` | SCAN 与多区域显示配置 |
| `m20_warehouse_inspection/` | 地图服务、校验和验收 Python 代码 |
| `docs/` | 架构、开发日志、测试报告及本文档 |
| `scripts/`、`tools/` | 构建、环境、地图和测试辅助工具 |
| `test/` | 配置、launch、资产及系统契约测试 |

### 3.2 `m20_warehouse_interfaces`：跨模块 typed 接口

- 目录：`src/m20_warehouse_system/common/m20_warehouse_interfaces/`
- 负责：仅定义 Action、msg、srv，不实现业务逻辑。
- 核心接口：楼层状态、局部感知状态、导航状态、任务状态、楼层切换、楼层导航、任务
  运行/控制以及外部跨层运输。
- 关键约束：导航目标与 `floor_id + generation` 绑定；切层失败后必须继续停车；外部
  电梯 provider 无权自行提交地图或释放 hold。
- 实机处理：**原样保留**。所有实机新模块优先适配现有 typed contract，只有确有缺失
  时才版本化扩展接口。

### 3.3 `m20_official_description`：官方 M20 机器人描述

- 目录：`src/m20_warehouse_system/common/m20_official_description/`
- 负责：包装云深处官方 M20 URDF、17 个 mesh、xacro 兼容入口和独立显示 launch。
- 来源：`deep_robotics_model` 固定 revision；可视模型未私自增加激光雷达或 IMU。
- 仿真作用：为 RViz/MuJoCo 联仿提供一致的 M20 运动树和外观参考。
- 实机处理：**保留**，但在上机前必须逐项核对真实设备的机型版本、关节名、TF、轮子
  方向、传感器外参和底盘尺寸。真实雷达应由独立传感器描述/静态 TF 挂接，不能修改
  官方 vendor 副本。

### 3.4 `m20_multifloor_map`：活动楼层局部感知数据面

- 目录：`src/m20_warehouse_system/navigation/m20_multifloor_map/`
- 节点：`m20_dynamic_local_sensing`、`m20_vendor_sensing_state`。
- 负责：只处理当前 `floor_id + generation`；切图时清缓存；裁剪/转发当前活动地图的
  局部点云；统计新代际点云是否新鲜并发布感知就绪状态。
- 仿真输入：活动全局云、`/m20/sim/body_pose` 和仿真 lidar pose。
- 输出：SCAN 使用的局部/传感器点云以及 generation-aware sensing state。
- 实机处理：**状态契约保留，数据源替换**。真实雷达驱动和点云预处理接管局部云；本包
  可继续负责楼层/代际门控，或将相同 contract 下沉至新的实机感知适配器。

### 3.5 `m20_inspection_core`：任务、切层和系统安全核心

- 目录：`src/m20_warehouse_system/safety_mission/m20_inspection_core/`
- 节点：
  - `m20_mission_executor`：按配置编排巡检点、切层和返回；
  - `m20_floor_switch_manager`：执行停车、reset、运输、地图 CAS、位姿交接和新鲜点云确认；
  - `m20_safety_supervisor`：急停、任务/切层 hold、地图/定位新鲜度和速度超时仲裁；
  - `m20_collision_guard`：使用 SCAN 在线占据点云生成内存栅格，对候选运动做前视双圆
    扫掠保护。
- 输入：导航候选速度、人工速度、急停/hold、里程计、`/grid_map/occupancy` 点云。
- 输出：唯一安全速度 `/m20/control/cmd_vel_safe`、任务/切层状态和停车状态。
- 实机处理：**保留，是上机安全主链之一**。需要接入真实急停、底层故障、定位健康和
  雷达近场保护，并根据实测制动距离重新标定；它不能替代机器狗固件级急停和避碰。

### 3.6 `m20_locomotion_control`：导航速度到 M20 SDK 的运动适配层

- 目录：`src/m20_warehouse_system/motion/m20_locomotion_control/`
- 节点：
  - `m20_navigation_adapter`：把 SCAN 横向跟踪项转换为滚动优先的偏航修正，完成方向
    模式、迟滞、限速、命令超时和可选速度反馈；
  - `m20_locomotion_manager`：只接收安全速度，执行最终 SDK 能力包线和 backend
    ready/fault 门控。
- 数据链：`cmd_vel_raw -> cmd_vel_candidate -> 安全层 -> cmd_vel_safe ->
  cmd_vel_sdk`。
- 能力配置：`config/m20_policy_v1_capabilities.yaml` 统一保存 M20 尺寸、速度限制、
  转弯与恢复约束。
- 实机处理：**保留并重点实测标定**。把速度反馈从仿真里程计改为真实融合里程计；将
  `cmd_vel_sdk` 接到真实 M20 官方运控接口；当前已新增 guarded basic_server 后端和
  MotionStatus 速度反馈，但仍需验证正反向、偏航符号、命令超时、控制频率和模式切换。
  禁止同时运行仿真与实机执行后端。

### 3.7 `m20_warehouse_sim`：RViz 平面运动学后端

- 目录：`src/m20_warehouse_system/simulation/m20_warehouse_sim/`
- 节点：`m20_rviz_kinematic_backend`。
- 负责：积分 `/m20/control/cmd_vel_safe`，发布 `/m20/sim/body_pose` 和 TF，让无动力学
  的 RViz 系统能够完成快速导航、任务和切层测试。
- 限制：不模拟轮腿接触、惯性、滑移、制动、负载或执行器能力。
- 实机处理：**生产启动时完全停用**；保留在回归测试环境中，不能与真实 odom/TF
  发布器同时运行。

### 3.8 `m20_mujoco_backend`：M20 动力学执行后端

- 目录：`src/m20_warehouse_system/simulation/m20_mujoco_backend/`
- 节点/工具：`m20_mujoco_backend`、`m20_generate_mujoco_world`。
- 负责：从同源地图 metadata 生成 MuJoCo 碰撞世界；执行 16 关节 `/JOINTS_CMD`；发布
  `/JOINTS_DATA`、`/IMU_DATA`、`/m20/sim/body_pose`、`joint_states`、TF 和接触/健康状态。
- 特性：SDK 站立稳定前不开放 backend ready；命令丢失时停止轮速并保持腿部稳定；
  可打开自动跟随原生三维窗口。
- 实机处理：**生产启动时完全停用**。继续作为算法/SDK 回归台架；真实底层接管关节
  反馈、IMU、状态估计和命令执行。

### 3.9 `m20_scan_planner`：适配后的唯一 SCAN 核心

- 目录：`src/m20_warehouse_system/navigation/m20_scan_planner/`
- 节点：`scan_planner_node`、`closed_loop_controller`；另有独立控制/测试工具。
- 负责：SCAN FSM、GridMap、路径搜索、B-spline 优化、实时重规划、轨迹跟踪、reset 和
  原版规划可视化。完整 M20 启动可显式注入双向轨迹跟踪扩展。
- 输入：活动楼层局部点云、lidar/body pose、目标和 reset/hold。
- 输出：B-spline/搜索可视化及 `/m20/navigation/cmd_vel_raw`。
- 实机处理：**保留算法主体**。将输入改为真实点云、TF 和状态估计；先保持冻结的原版
  SCAN 参数，再依据带时间戳的实机数据评估传感器延迟、重规划频率和跟踪误差。

### 3.10 `m20_scan_navigation`：仓库系统与 SCAN 的组合/协议层

- 目录：`src/m20_warehouse_system/navigation/m20_scan_navigation/`
- 节点：
  - `m20_navigation_gateway`：实现 `/m20/navigation/navigate`，校验目标楼层和地图代际，
    处理取消、超时和到点；
  - 项目新增的全局 occupancy A* 与顺序 SCAN 子目标已删除；
  - `m20_clearance_report`：净空诊断工具。
- launch：`f1_scan.launch.py` 组合 local sensing、SCAN 节点、闭环控制器和导航网关。
- 负责边界：不实现 SCAN 算法，不直接发布最终后端速度，也不依赖 MuJoCo/Gazebo。
- 实机处理：**保留导航网关和路线层**。实机 launch 中替换 CPU 仿真 local sensing，
  显式配置真实点云、位姿和 TF；路线层是否默认启用应由现场通道测试决定。

## 4. 十二个第三方构建依赖包

这些包属于当前 22 包闭包，但不是用户入口。其源码应继续保持锁定来源和补丁审计，不能
为了“合成一个包”复制到 `m20_warehouse_inspection` 中。

### 4.1 SCAN-Planner 依赖（10 个）

| ROS 2 包 | 工作空间目录 | 当前职责 | 实机处理 |
| --- | --- | --- | --- |
| `bspline_opt` | `src/third_party/SCAN-Planner/src/planner/bspline_opt/` | B-spline 轨迹优化、碰撞段 rebound | 保留算法库 |
| `path_searching` | `src/third_party/SCAN-Planner/src/planner/path_searching/` | GridMap 上的 A* 路径搜索 | 保留算法库 |
| `plan_env` | `src/third_party/SCAN-Planner/src/planner/plan_env/` | 点云栅格环境、占据与膨胀表示 | 保留；用真实点云验证更新频率与范围 |
| `traj_utils` | `src/third_party/SCAN-Planner/src/planner/traj_utils/` | 轨迹数学、B-spline 和可视化工具 | 保留算法库 |
| `scan_planner_msgs` | `src/third_party/SCAN-Planner/src/planner/scan_planner_msgs/` | SCAN 轨迹/规划消息定义 | 保留接口依赖 |
| `local_sensing_node` | `src/third_party/SCAN-Planner/src/simulator/local_sensing/` | 从全局 PCD 和仿真位姿渲染局部激光点云 | 实机生产停用，由真实雷达链替换 |
| `map_generator` | `src/third_party/SCAN-Planner/src/simulator/map_generator/` | 上游随机/仿真地图生成支持 | 实机生产停用，保留回归工具 |
| `mockamap` | `src/third_party/SCAN-Planner/src/simulator/mockamap/` | 上游仿真地图工具 | 实机生产停用，保留回归依赖 |
| `odom_visualization` | `src/third_party/SCAN-Planner/src/simulator/Utils/odom_visualization/` | 上游里程计/路径显示工具 | 生产可停用，调试时可保留 |
| `pose_utils` | `src/third_party/SCAN-Planner/src/simulator/Utils/pose_utils/` | 上游姿态与坐标工具 | 按依赖保留，不作为实机数据源 |

第三方树中还存在未进入当前构建闭包的 `plan_manage`、`go2_description`、
`waypoint_generator` 等目录。当前 M20 系统不应因其位于源码树中就自动启动或部署它们。

### 4.2 云深处 SDK 依赖（2 个）

| ROS 2 包 | 工作空间目录 | 当前职责 | 实机处理 |
| --- | --- | --- | --- |
| `drdds` | `src/third_party/sdk_deploy/src/drdds/` | `/JOINTS_CMD`、`/JOINTS_DATA`、`/IMU_DATA` 等 SDK/DDS 消息 | 保留，并核对实机固件/SDK 版本兼容性 |
| `m20_sdk_deploy` | `src/third_party/sdk_deploy/src/M20_sdk_deploy/` | 官方 M20 RL/ONNX 运控、状态机、ROS `cmd_vel` 桥和硬件/仿真接口 | 实机重点复核后保留；切换到真实 DDS/硬件接口，禁止修改模型和关节标定而不留审计记录 |

`src/third_party/sdk_deploy/` 下的 Lite3 包不属于 M20 当前闭包，也不应带入实机部署镜像。

## 5. 启动图中的模块组合

### 5.1 单场景自由导航

入口：`m20_warehouse_inspection/launch/f1_scan_rviz.launch.py`

主要组合：官方模型、地图服务、RViz 平面运动学、仿真 local sensing、SCAN、运动适配、
碰撞保护、安全 supervisor、RViz。RViz “2D Nav Goal” 最终进入
`/move_base_simple/goal`。

### 5.2 RViz 平面双区域完整巡检

入口：`m20_warehouse_inspection/launch/inspection_mission_rviz.launch.py`

在自由导航链上增加：活动楼层/代际、双区域总览、切层事务、typed 导航网关、任务执行器
及任务状态显示。机器人到 connector 并停稳后只切 active map，不改变连续位姿。

### 5.3 MuJoCo 完整动力学联仿

入口：`m20_warehouse_inspection/launch/inspection_mission_mujoco.launch.py`

保留同一地图、SCAN、任务和安全上层；停用 RViz 运动学后端，增加：

```text
m20_locomotion_manager
  -> m20_sdk_deploy/rl_deploy_cmdvel
  -> /JOINTS_CMD
  -> m20_mujoco_backend
  -> /JOINTS_DATA + /IMU_DATA + /m20/sim/body_pose
```

这条链最接近实机目标结构，后续主要替换最右侧 MuJoCo 状态/执行后端，并补齐真实雷达与
定位。

## 6. 仿真到实机的保留/替换矩阵

| 当前仿真能力 | 当前包 | 实机对应项 | 处理方式 |
| --- | --- | --- | --- |
| RViz 平面位姿积分 | `m20_warehouse_sim` | 真实 M20 状态估计/里程计 | 停用并替换 |
| MuJoCo 关节动力学与接触 | `m20_mujoco_backend` | 真实 M20 硬件、固件和 DDS | 停用并替换 |
| PCD 局部点云渲染 | `local_sensing_node`、`m20_multifloor_map` | 真实雷达驱动、去畸变、滤波、外参 | 替换数据源，保留楼层代际门控 |
| `/m20/sim/body_pose` | 两种仿真后端 | 融合定位 `/odom` + TF | 先适配兼容，后统一正式命名 |
| PCD 地图服务 | `m20_warehouse_inspection` | 实测地图与定位地图管理 | 保留契约，替换地图资产/定位接入 |
| SCAN 规划和轨迹跟踪 | `m20_scan_planner` | 实机导航算法 | 保留，基于实测时延/跟踪误差标定 |
| 滚动运动适配和 SDK 包线 | `m20_locomotion_control` | 实机 `cmd_vel` 到官方 M20 运控 | 保留并重点验收 |
| 碰撞与安全仲裁 | `m20_inspection_core` | 软件保护 + 硬件急停/固件保护 | 保留并增强，不能替代硬件保护 |
| 平面切图 `timed_hold` | `m20_inspection_core` | 电梯/升降平台 provider | 换成 `external_action` 等 transport adapter |
| `set_simulation_pose` | 仿真切层策略 | 电梯出入口重定位/地图定位确认 | 生产禁用，使用 `wait_for_target` 或正式定位交接 |
| 巡检任务与 typed Action | `m20_inspection_core`、`m20_warehouse_interfaces` | 真实巡检流程 | 保留，外接检测业务和站点系统 |

## 7. 当前默认隔离的源码目录

以下包已有源码，但通过 `COLCON_IGNORE` 排除在当前 22 包闭包之外：

| 包/目录 | 原因 | 实机迁移态度 |
| --- | --- | --- |
| `m20_foxy_nav_deploy` | 旧 ROS 2 Foxy 部署实验 | 不直接复用 |
| `m20_industrial_inspection*` | 早期 Gazebo/MuJoCo 原型 | 不重新并入主线 |
| `m20_nav2_gazebo_sandbox` | Nav2/Gazebo 沙箱 | 仅供对照 |
| `m20_motion_demo` | 独立运动演示 | 仅供台架调试 |
| `m20_lidar_bridge` | 早期实机雷达桥 | 候选复用；先审计消息、时间戳、TF 和 QoS |
| `m20_lidar_to_scan` | 早期 PointCloud2/LaserScan 实验 | SCAN 使用三维点云时默认不启用；按真实传感器需要评估 |

尤其不要仅通过删除 `m20_lidar_bridge` 的 `COLCON_IGNORE` 就把旧雷达节点加入生产图。
应先把它与当前 sensing contract、时间同步、外参和失联 fail-closed 规则对齐，再独立构建
和袋录回放验证。

## 8. 实机迁移时的推荐包边界

第一版实机系统建议仍由 `m20_warehouse_inspection` 单入口启动，运行时分为四层：

1. **设备与状态层**：真实 M20 SDK/DDS、激光雷达、IMU、关节状态、融合里程计和 TF；
2. **地图与导航层**：`m20_multifloor_map`、`m20_scan_planner`、
   `m20_scan_navigation`；
3. **运动与安全层**：`m20_locomotion_control`、`m20_inspection_core`；
4. **任务与集成层**：`m20_warehouse_interfaces`、`m20_warehouse_inspection`，以及后续
   巡检业务插件。

实机新增代码优先放在清晰的适配包中，例如真实雷达/定位适配器和真实电梯 provider，
不要写入第三方 SCAN 或 SDK 目录。只有对固定第三方源码的必要补丁才进入现有 patches
审计流程。

## 9. 本文档之后需要完成的实机迁移文档

本文只完成模块盘点，不代表当前系统已经可以直接上机。后续应依次形成并评审：

1. 实机硬件、网络、ROS_DOMAIN_ID、DDS 和 SDK/固件版本清单；
2. M20 真实运动接口与单一命令所有权设计；
3. 雷达型号、驱动、外参、时间同步、点云 QoS 和 SCAN 输入适配；
4. `map/odom/base_link` 定位与 TF 契约，以及 `/m20/sim/body_pose` 迁移办法；
5. 急停、失联、定位失效、雷达失效、运控故障和碰撞的 fail-closed 矩阵；
6. 单机台架、空旷场、单障碍、窄通道、完整巡检和多层运输的分级放行标准；
7. 仿真/实机 profile 隔离及一键启动、停止和日志归档规范。

在上述接口和安全评审完成前，不应把完整自主巡检命令直接发送到实机。
