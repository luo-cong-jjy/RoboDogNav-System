# 云深处 M20 平面双区域仓库巡检系统完整技术路线

> 现行方案更新（2026-08-11）：本文主体保留早期路线和验收过程，涉及
> `active_occupancy`、PGM/YAML、项目新增二维 A*、`use_grid_route` 或顺序短子目标的
> 内容均已撤销，不再代表运行时设计。现行接口以
> `docs/interface_contract.md` 和
> `docs/devlog/2026-08-11_pcd_only_native_scan_migration.md` 为准：地图资产仅为
> PCD/JSON，规划只使用原生 SCAN，独立安全守卫读取 SCAN 在线占据点云。

文档版本：1.7  
日期：2026-07-28  
适用阶段：RViz 平面双区域基线到 Gazebo、实机迁移  
主集成包：`m20_warehouse_inspection`

实施状态：阶段 0～5 及 0.7.0～1.3.0 校正已完成。当前系统已在同一 `z=0` 平面中
完成 F1/F2 原生 SCAN 导航、共享原点无瞬移 active PCD 切换、双场景四角
11 步巡检、任务级安全保持和独立工作空间构建。1.3.0 已通过
`config/rviz_v1_baseline.yaml` 冻结 RViz 稳定基线；新增的
`route_challenge_system.yaml` 是不进入冻结哈希的路线受扰动测试候选。

本文保留早期分离区域方案作为演进记录；凡与 1.1.0 相邻区域方案冲突之处，以第 24 节
和 `2026-07-28_connected_origin_map_switch.md` 为准。

## 1. 项目目标

建设一套基于云深处 M20 的仓库巡检系统。当前用同一水平面上的两块相邻区域近似一楼
和二楼，通过“到达共享原点、停止、原地切换地图、恢复任务”验证多楼层任务所需
的软件能力。

系统需要完成：

1. 在 RViz 中使用云深处官方 M20 模型。
2. 使用 SCAN-Planner 完成楼层内部导航、局部避障和轨迹优化。
3. 在一个 `z=0` 平面中同时显示两块同面积区域，分别表示 F1 和 F2。
4. 到达共享原点后可靠切换 active PCD，且保持机器人位姿连续。
5. 自动完成 F1 巡检、电梯切换和 F2 巡检。
6. 保留 RViz、Gazebo、MuJoCo 和实机 M20 的清晰后端边界。
7. 项目能够脱离当前大工作空间，在独立工作空间构建和运行。
8. 同步维护开发记录、接口文档、测试结果和第三方许可证。

## 2. 当前阶段明确不做

以下能力不属于当前 RViz 基线：

- 真实楼层高度建模。
- 楼梯、坡道、台阶和爬楼动作。
- 轮式、腿式和轮腿混合模式自动切换。
- 电梯门、按钮、网络协议和真实电梯调度。
- Gazebo 雷达和动力学同步。
- MuJoCo 与导航实时耦合。
- 直接控制 M20 关节。
- 未经低速、安全和人工接管验证的实机自主运行。

这些内容不是取消，而是在基础事务可靠后按独立阶段接入。

## 3. 已确认的核心建模决策

### 3.1 逻辑楼层不等于 z 高度

`F1`、`F2` 是逻辑楼层 ID。任务、地图和切换状态只能根据 floor ID 判断，不能根据
坐标或高度推断。

当前 RViz 仿真：

```text
F1 源地图：40 m × 40 m，本地坐标 [-20, 20] × [-20, 20]
F1 仿真偏移：(-20, 0, 0)
F1 显示范围：x=[-40, 0], y=[-20, 20], z=0

F2 源地图：40 m × 40 m，本地坐标 [-20, 20] × [-20, 20]
F2 仿真偏移：(20, 0, 0)
F2 显示范围：x=[0, 40], y=[-20, 20], z=0
```

两区在 `x=0` 相邻，F1 右围栏与 F2 左围栏在 `y=[-2,2]` 留门。机器狗只导航到门洞
中心 `(0,0)`；逻辑跨层仍必须经过 floor-switch 事务，不能靠越过 x 坐标改变楼层。

### 3.2 总览地图与规划地图分离

RViz 总览：

```text
/m20/visualization/all_floors_cloud
/m20/visualization/inactive_floors_cloud
```

前者始终包含两块区域，保留给分析工具；后者只包含当前非活动区域，是默认 RViz 使用
的概览。默认显示将其与使用原项目 `AxisColor` 的 active cloud 互斥绘制，避免共面
重复点随相机视角发生黄/粉色深度竞争。

规划地图：

```text
/m20/map/active_global_cloud
```

只包含当前 active floor。F1 导航时不允许 F2 点云进入 local sensing；F2 导航时也不
允许 F1 残留在 SCAN 占据地图中。

这样既能展示两块区域，也能验证实机未来必须具备的地图切换、缓存清理和重新定位过程。

### 3.3 仿真偏移只属于 simulation profile

平面左右偏移是当前可视化近似，不写入任务语义。未来实机可以让 F1/F2 都使用各自
局部 `map` 坐标，任务状态机和切换 Action 不需要改动。

## 4. 现有工作空间能力评估

### 4.1 主导航基础

`m20_scan_planner` 已包含：

- GridMap 局部三维占据地图。
- Dynamic A*。
- B 样条优化。
- SCAN 重规划 FSM。
- 开环/闭环控制器。
- M20 轻量运动学仿真。
- M20 JointState 和 RViz 显示。
- Mockamap、PCD 和 local sensing 启动链路。

推荐保留其算法主干，但重构外围接口和依赖。

### 4.2 当前动态换图障碍

现有 `map_generator/map_pub` 只在节点构造时读取一个 PCD。现有
`local_sensing_node` 在 `has_global_map=true` 后忽略所有新全局点云。因此以下操作
不足以实现切图：

```text
修改参数 file_name
重新发布另一个 PointCloud2
只改变 RViz 显示
```

正式实现必须让 local sensing 支持带 generation 的地图替换，并清空：

- `cloud_all_map`
- KdTree
- point hash
- normal cache
- 动态障碍缓存
- 当前 local cloud

### 4.3 当前 SCAN 重置障碍

`plan_env::GridMap` 有内部 `resetBuffer()`，但没有楼层切换服务。SCAN FSM、规划管理器
和闭环控制器也没有统一的 cancel/reset 确认。

需要新增：

- 导航 cancel。
- 执行 hold。
- 地图 reset。
- 当前轨迹失效。
- 新 generation ready。
- 恢复执行。

### 4.4 当前安全问题

现有 `cmd_vel_safety_mux_node` 可复用限速、超时和手动优先思想，但急停零速度仍会经过
普通加速度限制。正式安全层要求：

- e-stop 立即零速度。
- floor-switch hold 立即零速度。
- 定位失效立即零速度。
- 普通目标取消和命令超时可使用受控减速。
- SCAN 原始速度永远不能直接连接 SDK。

### 4.5 第三方选择

主线采用：

- 云深处官方 `deep_robotics_model` M20 资产。
- SCAN-Planner ROS 2 community 分支及必要算法包。
- PCL、Eigen、ROS 2 标准消息和 TF。

许可证按组件记录：SCAN 仓库根和所用 planner 包为 Apache-2.0，
`local_sensing_node` 的 package manifest 单独声明 GPL-3.0-only；不得用仓库根许可证
覆盖组件声明，具体见 `THIRD_PARTY_NOTICES.md`。

后续可选：

- RoboSense SDK：实机点云。
- Lightning-LM：建图与定位。
- M20 sdk_deploy：官方运动策略。

暂不采用：

- `3D-Nav-ROS2`：A1/CUDA/PCT 依赖重且存在使用限制。
- `TravExplorer`：本地只有资料，没有可集成实现。
- `core_planner`：没有 ROS 部署代码。
- RL 训练包：不属于多楼层软件事务范围。

## 5. 项目结构

项目最终由一个独立项目包集合交付：

```text
m20_warehouse_inspection       系统配置、bringup、总体验收
m20_warehouse_interfaces       Action、消息、服务
m20_official_description       官方 M20 模型及外围 wrapper
m20_multifloor_map             generation-aware local sensing
m20_inspection_core            任务、电梯、安全、后端适配
m20_locomotion_control         安全速度到 M20 RL SDK 的运动意图与接口适配
m20_warehouse_sim              RViz 运动学与未来 Gazebo 后端
m20_scan_planner               唯一 SCAN 核心、reset 与闭环控制
m20_scan_navigation            typed gateway、可选栅格路线、bringup
vendor/SCAN-Planner            固定版本算法依赖
```

拆包原则：

- 消息包不依赖业务实现。
- 模型包不依赖任务和规划。
- 导航包不依赖 Gazebo、MuJoCo 和 M20 SDK。
- 地图包不直接发送运动命令。
- 任务包不直接操作 SCAN 内部数据。
- 安全层是所有运动后端的唯一入口。
- bringup 只负责组合，不承载算法。

当前实现中，资产生成和预加载 map server 仍位于 `m20_warehouse_inspection`，因为它们
与集成配置、地图文件和显示 Marker 强关联；`m20_multifloor_map` 只拥有可被实物雷达
后端替换的 active-map-to-local-cloud 数据面。后续若地图服务独立部署，再把 server
机械迁入地图包，不改变接口。

## 6. 官方 M20 模型方案

模型源固定为云深处官方 `deep_robotics_model`：

```text
revision: 6113c62da96295e8d53abbc079af5296bf4649f8
license: BSD-3-Clause
```

处理原则：

1. 官方 URDF、惯性、碰撞几何和 mesh 原样保留。
2. RViz 基线直接加载官方 URDF，不增加雷达、IMU 或其他 link/joint。
3. 只允许在独立的后端 wrapper 中添加 Gazebo 插件，且不得改变官方 RViz 运动树。
4. 自动测试逐项比较官方源和交付副本的 link、joint、origin、parent/child 和 axis。
5. Gazebo 阶段才验证惯性、碰撞、接触和执行插件。

## 7. 地图资产方案

### 7.1 每层产物

每一层生成：

```text
floor_N.pcd       SCAN/local sensing 点云
floor_N.yaml      全局 A*/Theta* 占据栅格描述
floor_N.pgm       占据栅格图像
floor_N.json      障碍、边界、安全区和校验元数据
```

### 7.2 生成约束

- 尺寸：40 m × 40 m。
- `flat_multifloor` 稳定配置：每场景 180 个确定性障碍物。
- `dense_four_corner` 冻结配置：每场景 246 个障碍物，其中 240 个随机、6 个固定。
- `route_challenge` 实验配置：每场景 252 个障碍物，其中 240 个随机、12 个固定。
- 障碍高度：2 m。
- 点云表面分辨率：0.10 m。
- F1 seed：127。
- F2 声明 `replica_of: F1`，复用 seed 127。
- 地图边缘留出安全距离。
- 起点、电梯区、巡检点及其连接通道必须处于 free space。
- F1/F2 的内部 floor-local PCD、PGM、障碍布局和 intensity 严格一致。
- 两个场景使用 `simulation_offset=-20/+20 m`，在 `x=0` 接边。
- F1 `max_x` 与 F2 `min_x` 边界分别生成 4 m 门洞；其他围栏保持占用。
- 随机障碍物继续避开任务锚点和 native-SCAN 名义直达航段，避免随机种子偶然封死
  任务；固定障碍物允许在显式测试 profile 中进入名义航段。
- 冻结配置只让固定障碍物接近过道边缘；`route_challenge` 的固定障碍物明确截断
  左侧、底边、右边、顶边、斜向切换和最终返航名义直线，迫使局部规划器绕行。
- 正式 `dense_four_corner` 默认场景的所有独立障碍物统一满足至少 0.90 m 的
  本体边界间距；隔离的 `route_challenge` 与 `spacing_sweep` 保留各自实验阈值。
- 生成器输出相同输入时必须得到相同哈希。

### 7.3 地图验证

生成后自动检查：

- 文件存在且 PCD 头合法。
- 点数大于最低阈值。
- 边界符合 40 m × 40 m。
- reserved zone 内无障碍。
- 每个巡检点和电梯点位于 free cell。
- F1 所有任务点能到达 F1 电梯。
- F2 电梯出口能到达所有 F2 任务点。
- `replica_of` 楼层的内部 PCD、内部 PGM 和障碍 JSON 必须与源楼层完全相同；不同
  边界门洞允许完整文件哈希不同。
- 非副本 base floor 之间仍禁止意外出现相同 PCD 哈希。

## 8. 地图服务与代次

地图切换使用单调递增的 `map_generation`。

示例：

```text
启动 F1：generation=1
F1 -> F2：generation=2
F2 -> F1：generation=3
```

所有 ready/status 消息至少包含：

```text
floor_id
map_generation
stamp
ready
error_code
message
```

旧 generation 的确认不能解除新事务的 hold。

### 8.1 地图加载策略

第一版地图较小，可在启动时预加载并校验 F1/F2，切换时原子替换 active 数据。以后地图
变大时再支持 lazy loading 和缓存上限。

### 8.2 local sensing 代次替换

1.0.0 已恢复第三方 `local_sensing_node/pcl_render_node`。为了保留原射线与遮挡
效果，同时支持多楼层，仅增加默认关闭的地图重载开关：

1. 首层按第三方原逻辑建立法线、KD-tree、体素哈希和 360 度感知。
2. 收到下一张 active PCD 时清空所有旧地图派生缓存。
3. 用第二层 PCD 重建同一套原生索引。
4. 外围 `m20_vendor_sensing_state` 只统计当前 generation 的新鲜点云帧。

SCAN 内部 GridMap 不在 local sensing 中隐式修改，而是在 floor-switch 事务中通过
`/m20/navigation/reset` 显式清空。任何加载、reset、共享原点复核或新鲜感知确认失败
都保持 `FAULT_HOLD`。

## 9. 导航架构

### 9.1 独立诊断的原生 SCAN 路线

独立 `f1_scan.launch.py` 默认 `use_grid_route=false`，目标直接发送到
`/move_base_simple/goal`，用于与第三方项目进行算法和可视化对照。
GridMap、Dynamic A*、B 样条和闭环控制均由唯一的 `m20_scan_planner` 进程提供；
`m20_scan_navigation` 不编译算法副本。

### 9.2 自动任务全局路线

完整 M20 自动任务与人工目标默认先经过二维占据栅格净空 A*，再由原版 SCAN 执行
顺序子目标；这避免平台先进入没有滚动转弯空间的窄通道。相邻缓弯在硬净空与 M20
最小滚动半径校验通过后提前交接，急弯才停稳换段。后续仍可升级为带曲率状态的
Hybrid A*：

```text
目标点
  -> floor-aware global planner
  -> nav_msgs/Path
  -> /m20/navigation/global_route
```

该层只负责长距离引导和静态连通性，不直接控制机器人；日常完整系统默认启用，
`use_grid_route=false` 保留为原版 SCAN 对照入口。

### 9.3 SCAN 局部规划

原生模式直接接收目标；可选栅格模式沿用阶段 2 的 `navi_mode=1` 顺序短子目标：

```text
global_route
  -> 压缩后的 scan_goal
  -> 安全缓弯连续交接 / 危险急弯停稳交接
local PointCloud2
body_pose
  -> GridMap
  -> Dynamic A*
  -> B-spline optimization
  -> closed-loop tracking
  -> cmd_vel_raw
```

最初尝试把整条 `initial_path` 输入 `navi_mode=3`，但全局多项式会在狭窄拐角切角。
阶段 2 曾使用“膨胀 A* + 短子目标”组合；1.0.0 起日常人工与自动任务均恢复原生
SCAN 目标链。1.3.0 冻结的 11 步高密度任务已完成一次完整节点图验收。
`route_challenge` 保留相同任务与 SCAN 参数，只把 12 个静态障碍物放入名义航段，
用于后续绕障和压力回归；它不冒充已通过的稳定基线。若重新启用整条参考路径或栅格
短子目标，必须使用显式兼容 profile。

### 9.4 导航 Action gateway

任务层不通过反复发布 `PoseStamped` 和解析字符串判断结果。已实现
`/m20/navigation/navigate` typed Action gateway：

- 目标携带目标 ID、floor ID、map generation、`PoseStamped` 和超时。
- 接受前校验目标楼层与 active floor/代次一致且地图 ready。
- 每个目标开始前 reset SCAN；仅可选栅格模式再 reset A* 路线适配器。
- 发布距离、路线更新次数和人类可读原因的 typed feedback/state。
- cancel、pause 和超时都会停止目标并 reset 两级导航状态。
- 根据实际位置、路线事件和超时返回 typed result。
- 切图或代次变化时拒绝/中止目标。

路线状态使用事件队列消费，而不是只观察最新字符串快照，避免短暂的 `NO_ROUTE`、
`REJECTED_*` 或 `GOAL_REACHED` 被紧随其后的 `READY` 覆盖。

## 10. 运动与安全架构

```text
SCAN closed-loop controller
  -> /m20/navigation/cmd_vel_raw
  -> rolling navigation adapter
  -> /m20/navigation/cmd_vel_candidate
  -> collision guard
  -> safety supervisor
  -> /m20/control/cmd_vel_safe
  -> backend/final SDK gate
      -> RViz kinematic simulator
      -> M20 sdk_deploy
          -> MuJoCo/Gazebo joint bridge or real M20
```

当前 M20 ONNX 策略只接收前进、横移和偏航三维命令，不包含轮式/四足离散模式输入。
`m20_locomotion_control` 发布的巡航、协调转弯和横移是速度约束与诊断意图；16 个关节
如何协调仍由策略决定。需要强制步态时必须引入官方多策略或重新训练 mode-conditioned
策略，不能只靠修改 `Twist` 接口伪造。

官方策略每次同时生成 12 个腿关节位置目标和 4 个轮关节速度目标：腿部始终负责支撑、
姿态和策略学习到的转向协调，轮子负责滚动及左右轮速差。当前不存在“达到某阈值后
关闭轮子、改用纯四足踏步”的确定性分支。系统的高曲率转向判定为
`|yaw|>=0.25 rad/s` 且 `|vx|<=0.08 m/s` 或曲率不小于 `1.20`，此时只把前进速度
限制到 `0.12 m/s`，随后仍由同一个 ONNX 联合决定腿和轮动作。

安全输入：

- 人工急停。
- 手动命令。
- floor-switch hold。
- 定位健康。
- 地图 ready。
- 导航命令超时。
- 后端连接健康。
- 独立静态地图碰撞前视。

立即停车条件：

- e-stop。
- 切图 hold。
- active floor 与导航目标不一致。
- 地图 generation 不一致。
- 定位失效。
- 切图事务失败。

## 11. 电梯近似仿真

### 11.1 共享门洞不是楼层判定

M20 可以导航到两区相邻边的共享门洞，但不能用 x 坐标改变逻辑楼层。任务层到达原点
并停车后发起楼层切换 Action；规划器在任一时刻仍只加载一个 active floor。

### 11.2 状态机

```text
IDLE
  -> NAVIGATE_TO_SOURCE_LOBBY
  -> NAVIGATE_INTO_CABIN
  -> ACQUIRE_HOLD
  -> CANCEL_NAVIGATION
  -> WAIT_ROBOT_STOPPED
  -> RESET_NAVIGATION
  -> LOAD_TARGET_MAP
  -> VERIFY_SHARED_GATEWAY_POSE
  -> WAIT_MAP_READY
  -> WAIT_LOCAL_CLOUD_READY
  -> RELEASE_HOLD
  -> COMPLETE
```

任意状态失败：

```text
-> FAULT_HOLD
```

### 11.3 原地切图

connected profile 要求：

```text
source_trigger_pose = (0,0,0)
target_release_pose = (0,0,0)
teleport_on_floor_switch = false
```

floor-switch manager 不创建仿真位姿服务客户端。地图提交后从 odom 回读并确认机器狗
仍位于共享门洞；超出 0.35 m 容差即保持 `FAULT_HOLD`。旧分离区域测试 profile 才能
显式启用有确认的 `/m20/sim/set_pose`。

### 11.4 反向切换

状态机和配置从第一版就允许 F2 -> F1，虽然默认演示任务只走 F1 -> F2。这样能用于
连续切图稳定性测试。

## 12. 巡检任务

任务项是 typed step：

```text
inspection
elevator_transfer
dwell
wait_external
return_home
```

冻结的高密度四角任务为 11 步：

```text
F1 左下 -> 右下 -> 右上 -> 左上
-> 共享原点 (0,0) -> 原地切换 F2 active map
-> F2 左下 -> 右下 -> 右上 -> 左上
-> 共享原点 (0,0) -> 原地切回 F1 active map
-> 返回整个流程初始点 (-37,0)
```

`flat_multifloor_system.yaml` 保留早期七步回归任务；日常高密度入口使用
`dense_four_corner_patrol`。路线受扰动候选使用完全相同的 11 步语义和点位，仅将
mission ID 隔离为 `route_challenge_four_corner_patrol`。

已实现 `/m20/mission/run` Action。任务状态包含：

```text
mission_id
step_index
step_type
active_floor
map_generation
navigation_state
elevator_state
paused
fault
message
```

支持：

- start
- pause
- resume
- stop
- restart（通过新 RunMission goal 的 `restart=true`）
- retry_current_step

任务执行器只组合 typed `NavigateFloor` 和 `SwitchFloor` 子 Action，不直接发布 SCAN
目标，也不直接操作地图。暂停会取消当前导航子目标、reset 路线，并断言独立
`/m20/control/mission_hold`；恢复后从当前位置重新规划。原子楼层切换期间拒绝暂停。
导航或切层失败进入 `FAULT_HOLD`，只有 retry-current 或 stop 能改变故障等待状态。

## 13. RViz 显示

固定显示：

- 官方 M20 RobotModel。
- F1/F2 总览点云。
- active floor 高亮边界。
- 当前 local cloud。
- SCAN 占据/膨胀点云。
- 全局路径。
- B 样条轨迹。
- 历史轨迹。
- 巡检点和名称。
- 电梯入口、出口和当前状态。
- active floor、generation、任务状态和故障文本。

active map 保留 SCAN 原版 `AxisColor`，未激活场景副本使用固定蓝灰色，避免用户误认为
规划器同时使用两层。地图服务器不会在两个图层重复发布同一 active floor。

## 14. Profile 设计

### 14.1 `rviz_flat_regions`

强制依赖：

- 官方模型包。
- 地图、local sensing、SCAN 和任务核心。
- 运动学仿真。
- RViz。

禁止依赖：

- Gazebo。
- MuJoCo。
- drdds。
- m20_sdk_deploy。
- Lightning。
- RoboSense 驱动。

### 14.2 `gazebo_flat_regions`

在 RViz 基线上替换运动和传感器后端：

- Gazebo world。
- M20 Gazebo wrapper。
- Gazebo odom 和 cmd_vel。
- 初期仍允许 PCD local sensing。
- 后续再切 Gazebo ray/LiDAR。

### 14.3 `real_m20`

- 前后 RoboSense 点云。
- Lightning 或官方定位。
- M20 SDK adapter。
- 不允许仿真 teleport。
- 楼层切换改为定位后端重初始化并等待质量恢复。

## 15. 实机迁移接口

当前实机审计显示候选输入：

```text
/rslidar_points_front
/rslidar_points_rear
/IMU
/ODOM
/tf
/LOCATION_STATUS/MATCHING_ERROR
```

控制出口：

```text
/m20/navigation/cmd_vel_raw
  -> rolling navigation adapter
  -> /m20/navigation/cmd_vel_candidate
  -> collision/safety
  -> /m20/control/cmd_vel_safe
  -> final SDK gate
  -> m20_sdk_deploy
```

真机切层时：

1. 获取运动 hold。
2. 停止并取消导航。
3. 通知定位后端切换目标楼层地图。
4. 设置目标楼层电梯出口初值。
5. 等待定位质量、TF、点云时间戳和地图 generation 一致。
6. 人工或上层确认后恢复导航。

## 16. ROS 发行版策略

开发基线：

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Classic 11（后续阶段）
```

实机候选：

```text
Ubuntu 20.04
ROS 2 Foxy
```

策略：

- 核心接口优先使用 Foxy/Humble 都存在的标准消息和 rclcpp API。
- Humble 为主 CI。
- 独立增加 Foxy 构建清单和兼容补丁。
- 不把跨发行版 DDS 通信当作唯一部署方案。
- 若 SCAN 无法合理 backport，则在随车 Humble 计算机运行导航，通过稳定标准接口连接
  M20 Foxy 控制侧。

阶段 5 已完成 Python 3.8 语法和共同接口/API 的静态清单，但没有在 Foxy 原生环境构建
或运行。当前状态、风险项和目标机验收矩阵见
`docs/ros_distribution_compatibility.md`；在该矩阵通过前，Foxy 状态保持 `NOT RUN`。

## 17. 独立工作空间与依赖管理

独立构建流程目标：

```text
tools/prepare_isolated_workspace.sh /path/to/m20_warehouse_ws
校验两个固定 Git revision 与 CPU patch SHA-256
rosdep install --from-paths src --ignore-src -r
只 source /opt/ros/humble
colcon build --symlink-install --packages-up-to m20_warehouse_inspection
选择八个项目包执行 colcon test
```

要求：

- 所有 Git 依赖固定 commit，不使用浮动分支。
- `workspace_lock.yaml` 同时记录平台、项目版本、构建闭包和 patch hash。
- 不引用当前工作空间绝对路径。
- 地图和模型通过 ament index 定位。
- 不从其他业务包复制运行时文件。
- RViz 基线不能把 Gazebo/SDK 设为强依赖。
- 第三方本地修改必须形成可追踪 patch 或项目 fork。
- 目标目录必须为空，准备工具不得删除或覆盖已有工作空间。
- 1.3.0 稳定配置、PCD/PGM 和官方 M20 URDF 的哈希由
  `config/rviz_v1_baseline.yaml` 锁定；Gazebo 和路线挑战配置不得修改这些冻结文件。

## 18. 代码和文档规范

### 18.1 代码

- C++17。
- ROS 节点和状态机使用明确命名。
- topic、frame、timeout、速度限制均参数化。
- 不允许裸 magic number。
- 错误返回 typed code 和可读 message。
- 安全相关条件必须有测试。
- 注释说明原因、约束和状态，不重复语句本身。

### 18.2 文档

必须持续维护：

```text
docs/system_plan.md
docs/interface_contract.md
docs/devlog/YYYY-MM-DD_*.md
docs/test_reports/
CHANGELOG.md
THIRD_PARTY_NOTICES.md
dependencies.repos
```

每个阶段记录：

- 修改目标。
- 修改文件。
- 接口变化。
- 构建命令。
- 测试命令和结果。
- 已知问题。
- 下一阶段入口。

## 19. 测试体系

### 19.1 单元测试

- YAML schema 和跨字段校验。
- 楼层坐标转换。
- reserved zone 障碍排除。
- PCD 读取和边界。
- A*/Theta* 连通性。
- 电梯状态机正常、超时和失败路径。
- generation 旧确认拒绝。
- 安全立即停车。

### 19.2 节点测试

- map server 初始 F1 发布。
- F1/F2 切换及 QoS。
- local sensing 原子换图。
- SCAN reset。
- 仿真 set pose 确认。
- 导航 Action cancel。

### 19.3 无 GUI 集成测试

完整启动后自动执行：

1. F1 目标。
2. 到达电梯。
3. 切换到 F2。
4. F2 目标。
5. 任务结束。

断言：

- 切换期间所有安全速度为零。
- F2 active cloud 不包含 F1 专属探针障碍。
- `map_generation` 只递增一次。
- 机器人出现在配置的 F2 出口。
- 新 generation ready 前没有导航命令。

### 19.4 稳定性测试

- F1↔F2 连续切换 20 次。
- 每次切图后 local cloud 连续发布。
- 无旧 KdTree、occupancy 和 trajectory 残留。
- 无内存持续增长。
- 无重复节点名、TF 冲突或多个 cmd_vel 发布者。

### 19.5 人工 RViz 验收

- 两块区域同时可见且都在 z=0。
- M20 官方模型正常显示。
- active floor 高亮正确。
- 机器人不会穿过中间区域。
- 电梯传送过程状态清晰。
- F2 切换后障碍布局明显变化。

## 20. 性能和安全验收指标

RViz 第一版目标：

- local cloud 稳定达到 10 Hz。
- 不含模拟电梯等待时，85k～150k 点级 PCD 切换目标小于 2 s。
- e-stop 或 floor-switch hold 后 100 ms 内观测到零速度。
- 连续 20 次切图无失败。
- 任务完成率 10/10。
- 切图期间非零安全速度消息数量为 0。

完成状态：早期稳定配置已完成 20 次切图、任务 10/10、故障注入和 RSS 趋势测试；
冻结的 11 步高密度配置已完成一次完整节点图验收。`route_challenge` 尚未执行多轮
ROS 运行压力测试，必须单独记录结果，不能复用旧配置的 10/10 结论。

实机前必须新增：

- 定位质量阈值。
- 点云和 odom 超时。
- 物理急停与遥控接管。
- 低速上限。
- 网络断连保护。
- 独立碰撞监控。

## 21. 分阶段实施路线

### 阶段 0：边界与契约

工作内容：

- 创建 `m20_warehouse_inspection`。
- 固化本方案、接口和配置。
- 固定第三方版本。
- 建立 devlog、Changelog 和许可证记录。
- 单独构建新包，确认不影响现有包。

完成标准：

- 新包能被 colcon 发现和单独构建。
- 配置明确描述两块 `z=0` 区域。
- 不修改现有第三方和运行包。

### 阶段 1：地图与 RViz 双区域静态基线

状态：已于 2026-07-27 完成。实现结果和可复现验收记录见
`docs/devlog/2026-07-27_phase1.md`。

工作内容：

- 实现系统配置校验器。
- 实现双区域地图生成器。
- 生成 F1/F2 PCD、栅格和元数据。
- 实现 all-floors 与 active-floor map server。
- 创建 RViz 配置。

完成标准：

- RViz 同时显示 F1/F2。
- 两层都在 z=0。
- F1/F2 尺寸、点数和内部障碍严格一致，在 `x=0` 接边并生成对向门洞。
- active map 初始只包含 F1。
- 无 Gazebo/SDK 依赖。

### 阶段 2：官方 M20 与单层 SCAN 基线

状态：已于 2026-07-27 完成。实现与可复现验收见
`docs/devlog/2026-07-27_phase2.md`。

工作内容：

- 建立官方 M20 description 包。
- 启动 robot_state_publisher 和运动学仿真。
- 将 SCAN 输出改到 `cmd_vel_raw`。
- 建立第一版安全 supervisor 和 RViz 后端。
- 在 F1 完成目标导航。

完成标准：

- 官方模型可见。
- 单层导航、避障、停止工作。
- SCAN 不直接发布后端 `/cmd_vel`。

实际补充：

- 采用 active occupancy 膨胀 A* 与顺序短子目标，完成 12.54 m 有障碍路线。
- 增加独立前视碰撞保护和 fail-closed 速度安全 supervisor。
- 修复 SCAN 目标 0.2 m 内的无限重规划退出问题。
- 验证持续 `0.2 m/s` 输入下急停立即输出零速度。

### 阶段 3：地图热切换与重置事务

状态：已于 2026-07-27 完成。实现、故障修正和双向运行时验收见
`docs/devlog/2026-07-27_phase3.md`。

工作内容：

- local sensing 支持 generation 地图替换。
- GridMap 暴露安全 reset。
- SCAN FSM 支持 cancel/reset/hold。
- 运动学仿真支持有确认 set pose。
- 实现 floor switch Action。

完成标准：

- 手动 F1↔F2 切换成功。
- 切图期间速度为零。
- 新地图 ready 前不能恢复导航。
- 无旧地图残留。

实际补充：

- 新增 `m20_warehouse_interfaces`，固化 FloorState、LocalSensingState、SwitchMap、
  ResetNavigation、SetSimulationPose 和 SwitchFloor Action。
- 地图切换使用 expected generation compare-and-swap，成功提交后 generation 只增 1。
- SCAN reset 显式清理 GridMap、目标、路线、进度、失败状态和轨迹。
- 正反向事务都在新代次 local cloud 连续刷新后才释放安全 hold。
- F2 感知 X 范围全为正，切回 F1 后 X 范围全为负，证明无跨区域点云残留。

### 阶段 4：导航 Action 与巡检任务

状态：已于 2026-07-27 完成。实现、启动竞态修正和完整运行验收见
`docs/devlog/2026-07-27_phase4.md`。

工作内容：

- 将已实现的 floor-aware 栅格 A* 封装到 typed 导航 Action gateway。
- typed mission step 与任务执行器。
- 自动组合导航和电梯切换 Action。
- pause/resume/stop/retry 控制与任务级安全 hold。
- typed 导航/任务状态和 RViz Marker。

完成标准：

- 自动完成 F1_A、F1_B、E1、F2_A，并返回 F2 电梯大厅终点。
- pause/resume/stop/restart/retry 接口可用。
- 故障进入 FAULT_HOLD。

实际补充：

- 完成一次运行中的 pause/resume 验收，确认 `MISSION_HOLD` 且连续安全速度为零。
- 完成五步 F1→F2 自动任务，最终为 F2 generation 2，终点误差 0.001 m。
- 修复 map state 与 occupancy 到达顺序竞态，以及短暂路线事件被覆盖的问题。
- 为目标前路线 reset 增加 READY 屏障，旧路线和旧 SCAN 状态不会进入新 Action。

### 阶段 5：独立工作空间与回归

状态：已于 2026-07-27 完成。依赖闭包、回归设计、运行结果和已知边界见
`docs/devlog/2026-07-27_phase5.md`。

工作内容：

- 生成干净工作空间导入脚本。
- 完成 rosdep 和固定依赖。
- 增加 unit、launch、integration tests。
- 完成无 GUI 稳定性测试和报告。

完成标准：

- 不依赖当前工作空间 install。
- 干净构建和测试通过。
- 20 次切图稳定。
- 完整任务完成率 10/10。
- cancel、stop 和 FAULT_HOLD retry 故障矩阵通过。
- 多轮 RSS 增长和斜率在边界内。

实际补充：

- 固定七个项目包、13 包构建闭包、两个第三方 commit 和 CPU patch SHA-256。
- 在只含 `/opt/ros/humble` 的基础 overlay 中完成 13/13 独立构建。
- 20 次 F1↔F2 交替切换通过，F1 generation 1→21，保持后非零安全速度为 0。
- 10 次五步任务全部通过，共 50/50 step，最终 F2 generation 20。
- 非法切层、旧代次、导航取消、任务停止、错误楼层故障和 retry-current 全部通过。
- 10 轮 raw RSS 上界增长 76.3 MiB、斜率 5.72 MiB/轮，低于 128/8 阈值。
- 最终 RSS 选择器只统计同 launch 子进程，并区分首次事务预热与稳态趋势。
- 后续 1.3.0 高密度四角任务完成 11/11 单次节点图验收，并在全新独立工作空间完成
  当前 18 包闭包构建。
- `rviz_v1_baseline.yaml` 已锁定稳定配置、地图和官方模型哈希；路线挑战配置明确排除
  在冻结基线之外。

### 阶段 6：MuJoCo 完整动力学联仿

工作内容：

- 6A：审计官方 `sdk_deploy` 的 57 维观测、16 维动作、关节/IMU 接口和 MuJoCo
  桥；建立只消费 `cmd_vel_safe` 的 `m20_locomotion_control`。已完成。
- 6B：建立隔离的 `m20_mujoco_backend`，执行官方 16 关节 PD/前馈命令，回送
  `/JOINTS_DATA`、`/IMU_DATA`、物理 odom、TF 和关节状态。已完成。
- 6C：从 system YAML 指向的地图 JSON 同源生成 MuJoCo 双区域碰撞世界；两区始终
  同时存在，切图事务不重载物理世界、不传送机器人。已完成。
- 6D：增加 `motion_backend=external` 组合缝，完整入口关闭 RViz 平面运动学后端，
  启动 MuJoCo、官方 ONNX SDK、原版 SCAN、点云、任务和切图。已完成。
- 6E：加入 backend ready/fault 门控。SDK 复位全零控制字不能解除初始关节保持；
  完成站立并稳定后才开放 body pose/TF 和导航速度。已完成。
- 6F：执行台架与任务验收：静止、起立、直行、倒车、90°/180° 转向、障碍绕行、
  双向切图、11 步完整巡检和故障停车。近距离 SCAN、双向切图和 11 步主流程已通过；
  最后返航在旧 `180 s` 超时后经 `RETRY_CURRENT` 完成，MuJoCo 默认值已改为
  `300 s`。多轮转向、路线挑战和新超时无重试复验继续执行。
- Gazebo 不再是当前主路径。只有外部项目明确要求 Gazebo 插件、传感器或格式兼容时，
  才把同一 `/JOINTS_CMD` 执行契约移植过去；本项目完整系统验证直接使用 MuJoCo。

完成标准：

- PCD/occupancy、MuJoCo collision 和任务点来自同一 system YAML/JSON。
- 保持相同任务、地图切换和安全接口。
- MuJoCo 只执行官方 SDK 的关节命令；SDK 速度只来自 `cmd_vel_safe`。
- 后端切换不修改 `RunMission`、`NavigateFloor`、`SwitchFloor` 或 generation 语义。
- 无 GUI quick 导航、双向切层、完整任务和故障停止至少各通过一次。
- active floor 外点比例为零，点云/odom/物理后端失效均能触发安全停车。
- 记录静止漂移、转向扫掠包络、roll/pitch、接触数、最大力矩和实时率，未达标时不把
  MuJoCo profile 声明为实机等价。

### 阶段 7：实机适配

工作内容：

- RoboSense 前后点云。
- 定位和 TF。
- SDK 运动后端。
- 楼层定位切换。
- 人工接管和安全测试。

完成标准：

- rosbag 离线回放通过。
- 架空/低速测试通过。
- 定位、点云和控制失效均能停车。

## 22. 风险与处理

### SCAN 动态重置竞态

风险：旧 local cloud 在新地图 reset 后到达。  
处理：generation、hold、停止 local publish、确认 reset、原子换图、收到新 generation
连续点云后再恢复。

### 平面区域被误规划连通

风险：全局规划器通过中间空隙跨区。  
处理：全局规划器只加载 active floor 栅格，并校验目标 floor；总览地图禁止进入规划器。

### 仿真偏移污染实机地图

风险：任务点写死左右偏移坐标。  
处理：配置同时保存 floor-local 语义和 profile transform；任务首先按 floor ID 解析。

### 第三方未提交修改

风险：当前工作区 SCAN 内有未提交修改，无法独立复现。  
处理：阶段 5 已从固定 upstream commit 建立干净 clone；CPU 构建所需修改形成带
SHA-256 的独立 patch，其余 M20 适配位于项目自有 `m20_scan_planner`，协议和可选
路线适配位于 `m20_scan_navigation`。准备工具不
复制当前第三方工作树，因此本地未提交修改不会进入独立构建。

### ROS Foxy/Humble 差异

风险：Humble 通过但实机 Foxy 无法编译。  
处理：核心接口限制在共同 API；阶段 5 已完成 Python 3.8/共同接口静态审计但未声称
Foxy 已构建。优先采用随车 Humble 导航计算机和 Foxy 控制侧分层部署，原生 Foxy 状态
保持 `NOT RUN` 直至目标机矩阵完成。

## 23. 最终交付物

- 可独立构建的 ROS 2 项目包集合。
- 官方 M20 RViz/Gazebo description。
- F1/F2 双区域地图资产和生成器。
- active map server 与可靠切图事务。
- SCAN 导航适配。
- 安全 supervisor 和多后端适配。
- 自动双层巡检任务。
- RViz、后续 Gazebo 和实机 profile。
- 完整接口、开发日志、测试报告、依赖锁和许可证记录。

## 24. 当前执行点

阶段 0～5、0.7.0～1.3.0 校正已完成：平面 F1/F2 地图基线、无附加传感器链接的官方 M20、
RViz 运动学后端、原版 SCAN 首场景点云/规划/控制/可视化、闭环控制、独立碰撞保护、
fail-closed 速度链路、带 generation
的双向楼层切换，以及 typed 导航/任务 Action 均已通过构建、自动测试与端到端节点
验收；固定依赖的独立工作空间、20 次切图、任务 10/10、故障注入和 RSS 趋势也已通过。

0.8.0 曾将默认五步任务终点设为 F2 电梯大厅，并以 inactive-only 点云消除 RViz
共面颜色竞争；该分离区域终点已由 1.1.0 共享原点方案取代。

0.9.0 将原生控制器参数和 SCAN 点云/占据/轨迹显示属性与固定
`src/third_party/SCAN-Planner` 基线对齐。M20 几何、话题命名、楼层 reset、安全
supervisor 和自动任务全局引导是明确列出的集成边界，不改动 rebound/A*/B 样条数学主链。独立静态
碰撞保护按 profile 隔离：原生模式为 `0.25 + 0.05 m`，栅格增强模式为
`0.38 + 0.12 m`，避免增强层无意截停上游 SCAN 已判定可行的正常轨迹。

1.0.0 进一步取消命名空间和定制 local sensing 造成的行为差异：恢复
`/move_base_simple/goal`、`/quad_0/cloud`、`/grid_map/*`、`/planning/*` 及原版
RViz Display 树。自动任务默认也进入同一 SCAN 链。F1 quick navigation 和一次
F1→F2 原 renderer 重建均已通过图内验收。

2026-07-28 的 1.0.0 先将 F2 固化为 F1 副本并增加固定蓝灰色 inactive 图层。
1.1.0 随后把两区改为 `[-40,0]` 与 `[0,40]` 相邻布局：内部障碍继续完全复制，只有
F1 右边界和 F2 左边界分别开 4 m 门洞。启动即发布两场景，自动任务先自行到达共享
原点 `(0,0)`，停车后只切 active map，不创建或调用仿真传送客户端。最终五步回归达到
F2 generation 2，切换跳变 `0.000 m`，终点误差 `0.142 m`。

1.2.0 将 F2_B 纳入默认巡检，并增加 `transit` 类型的 F2_RETURN_VIA_A 回程点。当前
默认任务为七步，F2 覆盖与 F1 相同的 A/B 巡检点后返回其进入点 `(0,0)`。重新生成
地图后的 PCD/PGM 哈希与 1.1.0 完全一致，障碍数量、布局和密度未变化。

1.3.0 增加隔离的高密度四角配置：每场景 246 个障碍物，F1/F2 均按左下、右下、
右上、左上巡检，随后反向切回 F1 并返回全流程起点。完整 11 步节点图达到切换跳变
`0.000 m`、最终误差 `0.118 m`；稳定配置、地图和官方 URDF 已写入
`rviz_v1_baseline.yaml` 冻结。

冻结后新增的 `route_challenge_system.yaml` 不修改 1.3.0 基线。它保持相同双场景、
任务、SCAN 和安全参数，每场景使用 240 个随机障碍物加 12 个进入名义航段的固定
障碍物。静态验收已经证明 7 类任务航段均需要绕行、实际最小障碍间距
`0.702172 m`，且按 0.30 m 安全轮廓膨胀后任务点仍连通；完整 ROS 多轮回归待执行。

0.7.0 将项目源码边界扩为八个包，`colcon list --packages-up-to` 的实际闭包为 18 包；
模型/导航校正实现和当前工作区测试证据见
`docs/devlog/2026-07-27_model_scan_correction.md`。阶段 5 的 7/13 数字保留为当时版本
的历史结果，不代表 0.7.0 当前闭包。

阶段 6A～6E 已完成：官方 16 关节模型、ONNX SDK、MuJoCo 接触动力学、同源双区域
collision、原生 PCD local sensing、SCAN、typed 任务和楼层事务已组成一个完整入口。
0.70 m 高密度场景生成 492 个障碍实体和 8 段围栏；后端稳定站立后 `z≈0.564 m`，
近距离 SCAN 目标从 `(-36.78,-0.02)` 到达 `(-34.51,0.11)`，后端实时率约
`0.9996` 且无物理故障。

6F 已完成一次 0.70 m 高密度主流程：F1 四角、物理行走至原点切 F2、F2 四角、
物理返回原点切回 F1，最终回起点 `(-37.0449,0.0728)`，误差 `0.0855 m`，任务
11/11。旧 `180 s` 单步预算使最后返航先超时，随后 `RETRY_CURRENT` 成功；已把
MuJoCo profile 独立改为 `300 s`。原生 viewer 已增加 4 m 默认跟随相机；停车轮端
制动把 30 s 平面漂移降至 `0.0000158 m`，同时短时运动测试确认可正常释放并重新
接合。当前余项为新超时无重试复验、60 s/多轮停车统计、90°/180° 重复转向台架和
路线挑战。一次长任务仍不替代多轮稳定性和实机等价性结论。

2026-07-29将正式`dense_four_corner`默认障碍物本体间距提高为0.90 m。F1/F2
仍各含246个障碍物，实际最近边界距离均为`0.900236 m`；0.30 m栅格膨胀后起点、
四角任务点和共享门洞保持连通，包级149项回归通过。历史0.70 m完整MuJoCo结果
不迁移为新资产的动态结论，0.90 m默认场景仍需一次无重试11步复验。

自由导航轮腿基线已完成六个无障碍威胁目标，6/6成功且无碰撞停止。官方ONNX在
同一推理周期同时产生12个腿部位置目标和4个轮速目标，轮腿不是互斥模式。基线中
63.81%的运动采样进入`LATERAL_MANEUVER`，两个180°回程的横向扫掠达到
0.564～0.611 m。

滚动适配候选已完成并设为默认：仅在`NAVIGATION`来源下把SCAN侧向跟踪量转换为
航向修正，加入协调转向进入/退出迟滞、巡航偏航软死区/低通和输出变化率限制；
`MANUAL`来源继续保留横移，所有硬停止保持立即清零。SCAN规划/控制数学主链、
官方ONNX模型和16维动作定义均未修改。相同六目标MuJoCo A/B为6/6成功、无碰撞
停止，自动`LATERAL_MANEUVER`从63.81%降至0%，总耗时175.59 s降至101.59 s，
平均终点误差0.162 m降至0.092 m，平均横向偏离0.274 m降至0.193 m，模式切换
次数50降至25。该结果放行滚动适配为当前自由导航默认值，但仍不替代0.90 m完整
11步巡检、多轮耐久与实机转向净空验收。

2026-07-30 将滚动适配从安全层之后前移到独立
`m20_navigation_adapter`：SCAN 原始速度先形成
`/m20/navigation/cmd_vel_candidate`，碰撞保护和 safety supervisor 再共同检查该
candidate，末端 locomotion manager 禁止二次适配。安全层新增锁存
`/m20/control/execution_hold`；闭环控制器和 SCAN FSM 同步冻结 B 样条时间，恢复
规划从 odom 位置和零导数起步。碰撞保护改用与 SCAN 一致的 `±0.18 m` 前后双圆并
输出首个碰撞样本诊断。正式密集场景同一目标在 RViz 与 MuJoCo 均到达，终点误差
分别为 0.164 m 和 0.060 m；两轮均未出现内部 A* 失败、前三控制点入障碍或运行期
碰撞停车。

Gazebo 路线降为可选兼容项，不再阻塞当前完整系统验证。
