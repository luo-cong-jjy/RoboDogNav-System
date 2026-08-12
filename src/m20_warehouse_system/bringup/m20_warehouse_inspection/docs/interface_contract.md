# 系统接口契约

本文定义模块间稳定边界。阶段 1 以后新增节点时应遵循这些名字和语义；若确需变更，
必须同步更新方案、配置、测试和开发日志。

## 1. 坐标系

```text
warehouse_overview
  仅用于 RViz 同时显示 F1/F2 两块平面区域

map
  当前 active floor 的规划坐标系

odom -> base_link
  仿真或实机运动估计链路

base_link -> lidar_link
  雷达外参
```

当前平面仿真中，`map` 和 `warehouse_overview` 数值可以对齐。实机模式允许各楼层重新
使用自己的局部地图坐标，不允许任务层通过 x 坐标猜测楼层。

当前 connected profile 中，F1 为 `x=[-40,0]`、F2 为 `x=[0,40]`，共享原点
`(0,0)`。该布局只用于 RViz 平面近似；逻辑楼层仍必须以 `floor_id + generation`
判定。

## 2. 楼层与地图

```text
/m20/map/state
  m20_warehouse_interfaces/msg/FloorState
  floor_id、generation、ready、phase 和 message 的原子状态快照。

/m20/map/active_floor
  std_msgs/msg/String
  FloorState 的兼容性拆分话题；新模块应优先使用 typed 状态。

/m20/map/generation
  std_msgs/msg/UInt64
  只在新 active map 成功提交后递增一次。

/m20/map/ready
  std_msgs/msg/Bool
  当前 floor/generation 是否完成提交。

/m20/map/switch
  m20_warehouse_interfaces/srv/SwitchMap
  请求包含 target_floor 和 expected_current_generation。

/m20/map/active_global_cloud
  sensor_msgs/msg/PointCloud2
  仅包含当前楼层，供 local sensing 使用。

/m20/visualization/all_floors_cloud
  sensor_msgs/msg/PointCloud2
  始终包含 F1/F2，供分析工具使用，禁止接入规划器。

/m20/visualization/inactive_floors_cloud
  sensor_msgs/msg/PointCloud2
  只包含当前非活动楼层，供默认 RViz 与黄色 active cloud 互斥绘制。

/m20/visualization/floor_markers
  visualization_msgs/msg/MarkerArray
  楼层边界、名称和 active/inactive 状态，仅用于显示。
```

地图服务器启动时预加载并校验所有地图。切换服务先校验 expected generation 和目标
楼层，再发布目标楼层、下一代次、`LOADING/ready=false`；准备完成后一次提交目标 PCD
cloud、marker 和 `READY/ready=true`，generation 递增。generation 不匹配、未知
楼层和重复目标都会拒绝，不允许旧事务覆盖新状态。

启动时立即发布 active、inactive 和 all-floors 三份锁存快照，因此 RViz 首帧即可看到
两个场景。规划链只允许订阅 active cloud；碰撞守卫使用 SCAN 根据活动 PCD 与实时雷达
生成的在线占据点云。

普通 `PointCloud2` 只承担点数据传输；消费者必须通过 typed state 把点数据绑定到
`floor_id + generation`，不能通过点坐标推测楼层。

默认 RViz 不叠加 `all_floors_cloud` 和 `active_global_cloud`，避免完全共面的同一批
点发生深度竞争。active floor 固定显示黄色，inactive floor 固定显示蓝灰色；只有楼层
状态切换会交换颜色，移动相机不会改变颜色。

## 3. 局部感知

```text
/m20/sensing/local_cloud
  sensor_msgs/msg/PointCloud2
  当前 active floor 的局部点云，供 SCAN GridMap 与 RViz 使用。

/m20/sensing/lidar_pose
  geometry_msgs/msg/PoseStamped
  与仿真 odom 对齐的雷达位姿。

/m20/sensing/state
  m20_warehouse_interfaces/msg/LocalSensingState
  floor_id、generation、ready、fresh_cloud_count、point_count、XY 边界和 message。
```

收到 `FloorState.ready=false` 时，local sensing 立即清空旧全局缓存、旧 local cloud 和
新鲜帧计数。只有收到 READY 状态以及对应的新 active cloud 后才恢复发布。单帧最多
3500 点，采用轮转切片覆盖完整局部集合，防止 DDS 大分片丢失；输出 QoS 为 reliable、
volatile。楼层事务以切换后基准计数再增加 3 帧作为恢复条件。

## 4. 导航

```text
/move_base_simple/goal
  geometry_msgs/msg/PoseStamped
  RViz/人工目标入口，直接进入原版 SCAN。自动任务通过 typed Action 进入 gateway，
  gateway 也只向该原生目标入口发布。

/m20/navigation/navigate
  m20_warehouse_interfaces/action/NavigateFloor
  typed 导航入口；goal 绑定 goal_id、floor_id、map_generation、目标位姿和超时。

/m20/navigation/state
  m20_warehouse_interfaces/msg/NavigationState
  当前 Action 的 goal/floor/generation、状态、剩余距离和原因，锁存。

/m20/navigation/reset
  m20_warehouse_interfaces/srv/ResetNavigation
  请求绑定 floor_id + generation；清空 SCAN GridMap、目标、轨迹、进度和失败状态。

/grid_map/occupancy
  sensor_msgs/msg/PointCloud2
  SCAN 根据当前活动 PCD 与局部雷达实时生成的占据体素。碰撞守卫只在内存中把该点云
  栅格化做前视扫掠；它不是磁盘地图，也不是 nav_msgs/msg/OccupancyGrid。

/m20/navigation/cmd_vel_raw
  geometry_msgs/msg/Twist
  SCAN 控制器原始输出，禁止直接连接 M20 SDK。

/planning/tracking_direction
  std_msgs/msg/String
  闭环控制器当前使用的 FORWARD 或 REVERSE 轨迹执行方向。完整 M20 系统依据
  capability profile 启用双向选择；独立 SCAN 启动默认固定为 FORWARD。方向只在
  接收新 B-spline 后依据多点切向更新，REVERSE 仅用于后轴对齐的近似直线轨迹，
  不能把右后方曲线解释为倒车。该话题只用于诊断，不改变世界系 B-spline、导航
  目标或地图坐标。

/m20/navigation/cmd_vel_candidate
  geometry_msgs/msg/Twist
  将 SCAN 全向跟踪量转换为 M20 滚动/转向语义后的自动导航候选速度。碰撞保护与
  safety supervisor 必须共同使用该话题，保证预测命令和后端实际候选命令一致。
  启用实测速度闭环时，有界补偿在发布该话题前完成，因此补偿后的完整运动仍须先经过
  碰撞预测，不允许在 safety supervisor 之后追加未检查的速度。

/m20/navigation/candidate_mode
  std_msgs/msg/String
  预安全适配器的 STOPPED、WHEEL_CRUISE 或 COORDINATED_TURN 诊断。

/m20/navigation/velocity_feedback_state
  std_msgs/msg/String
  有界机体系速度闭环 JSON 诊断：enabled、active、reason、measurement_age_sec、
  reference、measurement、error、correction、integral 和 output。ODOMETRY_STALE、
  ODOMETRY_NONFINITE、ODOMETRY_FRAME_MISMATCH 或 EXECUTION_HOLD 时补偿必须清零，
  输出退回未经反馈增加的滚动适配候选。当前实验闭环仅在 WHEEL_CRUISE 且
  |wz_ref|<=0.08 rad/s 时可激活；转弯、横移和保持阶段 reason=MANEUVER_GATED
  并清空积分。
```

人工与自动任务统一使用
`/move_base_simple/goal -> SCAN rebound/B-spline -> closed-loop controller`。
系统没有离线 PGM/YAML、外部二维 A*、顺序短子目标或 route reset。导航成功只校验
XY，默认容差 0.20 m；目标 PoseStamped quaternion 不作为严格终点 yaw。完整 M20
系统可按能力配置启用控制器的 FORWARD/REVERSE B-spline 执行，但不修改世界系路径。

碰撞守卫发现当前/预测足迹危险时，安全层先停车并执行有界滚动恢复。恢复预算耗尽或
无安全恢复解时，gateway 清空旧 SCAN 状态，等待在线点云守卫连续恢复 `CLEAR`，再将
同一最终目标重新交给原生 SCAN；超过配置次数则 fail closed 返回 NO_ROUTE，不切换到
另一套规划算法。

reset 时先发布驻车轨迹，再调用 GridMap `resetBuffer()` 并回到 `WAIT_TARGET`。导航
Action 接受目标前校验 ready floor/generation；取消、超时、楼层变化和失败均 reset
原生 SCAN。需要强制终点朝向的任务必须另行实现“可转身区域 -> 位置目标 -> 朝向对齐”
的分阶段规划，不能在狭窄通道内隐式要求原地掉头。

## 5. 巡检任务

```text
/m20/mission/run
  m20_warehouse_interfaces/action/RunMission
  运行系统配置中的 typed mission sequence；同一时刻只允许一个活动任务。

/m20/mission/control
  m20_warehouse_interfaces/srv/ControlMission
  PAUSE=1、RESUME=2、STOP=3、RETRY_CURRENT=4。

/m20/mission/state
  m20_warehouse_interfaces/msg/MissionState
  mission、step、active floor/generation、paused、fault 和原因的锁存状态。

/m20/visualization/mission_marker
  visualization_msgs/msg/Marker
  RViz 任务步骤、楼层、generation 和故障文本。
```

默认 sequence 为 F1_A、F1_B、E1:F1→F2、F2_A、F2_B、
F2_RETURN_VIA_A、F2_TERMINAL。切层步骤先导航到共享原点 `(0,0)`；F2 完成与 F1
对应的 A/B 两点巡检后，经 A 点返回该原点。任务执行器只调用 typed `NavigateFloor`
和 `SwitchFloor` 子 Action；不直接操作 SCAN、地图或仿真位姿。
inspection step 导航成功后执行配置的 dwell。`floor_transfer` step 先导航 approach，
再导航 connector 触发位姿，最后调用楼层切换 Action。旧的 `elevator_transfer` 仍兼容。
transit 和 terminal step 只导航，不执行巡检
dwell。

pause 会取消活动导航并保持任务级停车；resume 从当前位置重新规划。原子切层期间拒绝
pause。子 Action 失败进入 `FAULT_HOLD`，retry-current 重试当前 step，stop/cancel
结束任务但保留安全 hold。新任务的 `restart=true` 只表示允许在上一个任务已结束后
重新执行，不允许并发任务。

## 6. 安全与运动后端

```text
/m20/control/cmd_vel_manual
  geometry_msgs/msg/Twist

/m20/control/e_stop
  std_msgs/msg/Bool

/m20/control/floor_switch_hold
  std_msgs/msg/Bool

/m20/control/mission_hold
  std_msgs/msg/Bool
  pause、stop、cancel 和任务故障时的独立锁存停车请求。

/m20/control/collision_stop
  std_msgs/msg/Bool
  独立碰撞前视保护；true 时安全 supervisor 立即停车。

/m20/control/safety_state
  std_msgs/msg/String
  当前安全门控或命令来源，transient_local。

/m20/control/collision_guard_state
  std_msgs/msg/String
  NOT_READY、CLEAR 或 COLLISION_STOP，transient_local。

/m20/control/collision_guard_diagnostic
  std_msgs/msg/String
  首个碰撞样本的当前/预测类型、运动模型、时间、前/后圆、坐标、栅格、原因、
  候选速度，以及可选恢复/预算耗尽原因。`RASTER_SHELL_ESCAPE` 表示只有保守栅格
  半单元外壳命中、hard-body 扫掠仍自由的有界直线脱离；
  `MAP_EDGE_INWARD_RECOVERY` 表示从 active occupancy 边缘向地图内部执行的有界
  零偏航直线恢复。两者都不是实体碰撞后的穿障许可。

/m20/control/execution_hold
  std_msgs/msg/Bool
  safety supervisor 的锁存执行保持；除 NAVIGATION 外均为 true。SCAN 控制器和
  FSM 共同用它冻结 B 样条执行时间。

/m20/control/cmd_vel_safe
  geometry_msgs/msg/Twist

/m20/locomotion/cmd_vel_sdk
  geometry_msgs/msg/Twist
  SDK 专用速度，只能由 cmd_vel_safe 派生。滚动适配已在安全层之前完成，末端
  locomotion manager 只做 SDK 包络和后端 ready/fault 门控，禁止二次适配。

/m20/locomotion/mode
  std_msgs/msg/String
  STOPPED、WHEEL_CRUISE、COORDINATED_TURN 或 LATERAL_MANEUVER。
  该值是上层速度意图，不是官方 ONNX 的离散步态输入。

/m20/sim/body_pose
  nav_msgs/msg/Odometry
  统一运动估计输出；MuJoCo、RViz 运动学或实机 profile 通过同一语义接口替换。
  pose 位于 header.frame_id（MuJoCo 默认 world），完整 twist 位于
  child_frame_id（base_link）：linear.x/y/z 是机体系实测线速度，
  angular.x/y/z 是机体系实测角速度。控制器禁止把 pose 差分得到的世界系速度直接
  当成机体系反馈。

/m20/sim/dynamics_state
  std_msgs/msg/String
  MuJoCo 诊断 JSON。新代码应使用 base_linear_velocity_world、
  base_linear_velocity_body 和 base_angular_velocity_body 三个显式坐标字段；
  base_velocity 是为历史测试保留的“世界系线速度 + 机体系角速度”混合字段，禁止
  作为新控制器输入。

/m20/sim/set_pose
  m20_warehouse_interfaces/srv/SetSimulationPose
  旧分离区域仿真/测试 profile 的可选平面传送；当前 connected profile 不创建客户端。
```

急停、切图保持、任务保持和路线段间保持必须立即输出零速度，不经过普通减速度限制。自动链路固定为
`cmd_vel_raw -> cmd_vel_candidate -> collision/safety -> cmd_vel_safe`；RViz 和
M20 SDK 后端只执行 `cmd_vel_safe` 派生速度。安全保持时 closed-loop controller
冻结自身执行时钟但继续提供待检查 candidate；SCAN FSM 同步平移局部轨迹时间，恢复
规划从实际 odom 位置和零速度/加速度起步，不继承未执行的旧轨迹导数。碰撞保护在
map ready=false 时 fail-closed。实机后端必须拒绝 `SetSimulationPose`，并以定位
后端的地图切换和初值确认取代传送。当前 connected profile 的正常楼层事务只切地图，
不改变 odom。

SDK 控制链使用 `/JOINTS_CMD`、`/JOINTS_DATA` 和 `/IMU_DATA`。仿真或实机任一时刻
只能存在一个关节命令发布者和一个执行后端。Gazebo 必须实现 16 关节动力学和 IMU
反馈后才可启动 SDK；RViz 平面运动学后端不满足这一条件。

`/m20/locomotion/cmd_vel_sdk` 的三个有效分量为机体系前进速度 `linear.x`、横移速度
`linear.y` 和偏航角速度 `angular.z`。SDK 桥将三者写入 `UserCommand`，官方 ONNX
策略约每 20 ms 将它们和 IMU、16 关节状态、上一帧动作拼成 57 维观测，并联合输出
16 维动作：12 个腿关节位置目标和 4 个轮关节速度目标。因此 `linear.x != 0` 且
`angular.z != 0` 表示连续滚动曲线；腿和轮可以同时参与。上层
`WHEEL_CRUISE/COORDINATED_TURN` 只用于限幅、碰撞预测、驻车和诊断，不是 ONNX
步态选择开关。最终 `/JOINTS_CMD` 每个关节均使用
`[kp, position, kd, velocity, torque_ff]` 契约；腿关节主要执行位置 PD，轮关节强制
`kp=0` 并执行速度/阻尼控制。

## 7. 楼层切换

楼层切换采用 Action，而不是一次性 topic：

```text
goal:
  source_floor
  target_floor
  elevator_id  # 兼容字段；语义为 connector id

feedback:
  phase
  map_generation
  message

result:
  success
  active_floor
  map_generation
  error_code
  message
```

Action 名称为 `/m20/floor_switch`，锁存的人类可读阶段同时发布到
`/m20/floor_switch/state`。已实现的执行顺序是：

```text
VALIDATE_TRIGGER
  -> HOLDING
  -> RESETTING_OLD_NAV
  -> TRANSFERRING
  -> SWITCHING_MAP
  -> HANDING_OFF_POSE
  -> RESETTING_NEW_NAV
  -> WAITING_SENSING
  -> COMPLETE
```

每个 connector 可配置任意数量的有向 `transitions`，每条 route 独立定义 source trigger、
target release，并可覆盖运输和位姿策略。hold 后连续 3 个样本同时满足
`cmd_vel_safe` 和 odom 停止阈值，事务才会继续。地图切换采用 generation
compare-and-swap；新楼层至少出现 3 个 reset 后的新鲜 local cloud 才释放 hold。
默认 connected profile 使用 `timed_hold + preserve`，地图提交后校验 odom 仍位于
共享原点容差内，既不发布位姿，也不调用 `/m20/sim/set_pose`。

`external_action` 通过 `/m20/floor_transfer/execute` 调用
`ExecuteFloorTransfer` provider。provider 只负责物理运输，不拥有地图 generation、
导航 reset 或 safety hold。pose handoff 独立选择 `preserve`、
`set_simulation_pose`、`wait_for_target` 或 `none`；因此替换真实电梯、升降平台或其他
方案不会改动楼层事务状态机。

事务开始后的取消、服务失败、超时或异常都保持 `floor_switch_hold=true`。故障状态只
允许人工诊断、重试或停止任务，不允许继续发送导航速度。事务开始前的无效请求和
`ERROR_NOT_AT_TRIGGER` 不改变地图，也不接管已有安全状态。

## 8. QoS 约定

```text
FloorState、LocalSensingState、NavigationState、MissionState、地图、栅格、
Marker、安全状态
  reliable + transient_local + depth 1

local_cloud
  reliable + volatile（SCAN/RViz best-effort 订阅兼容）

odom、cmd_vel、动态路径
  volatile，按各节点配置的实时 depth
```

切换正确性只依据 typed state 和服务/Action 确认，不依赖 transient-local 点云的到达
顺序，也不依赖字符串日志。
