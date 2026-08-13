# 接口约定

本文档记录工业巡检项目与官方 Deep Robotics 例程之间的接口边界。

## 归属边界

本项目拥有：

- `m20_industrial_inspection`
  - 巡检任务、安全仲裁、运动适配层。
  - 项目化 MuJoCo 仿真入口 `m20_factory_simulation`，用于在工厂场景中扮演 M20 硬件。

外部或可替换能力提供者：

- `sdk_deploy`：官方运动控制部署包，保留在 `src/third_party/sdk_deploy`，由 `m20_sdk_deploy/rl_deploy_cmdvel` 消费 `/cmd_vel` 并生成 `/JOINTS_CMD`。
- `deep_robotics_model`：机器人模型资源。
- `lightning-lm-deep-robotics`：建图与定位例程。
- 真机 SDK 或出厂程序：接口可能与当前下载例程不同。

项目包不复制 `sdk_deploy`。二者只通过 ROS 话题和 ROS 包依赖连接。

## 项目仿真接口

本项目提供自己的 MuJoCo 仿真入口：

```text
m20_factory_simulation
```

它复用官方控制器需要的低层话题：

```text
发布 /IMU_DATA
发布 /JOINTS_DATA
订阅 /JOINTS_CMD
```

因此当前官方 `rl_deploy_cmdvel` 可以继续使用，不需要知道场景来自官方还是本项目。

完整仿真控制链路：

```text
/m20_inspection/cmd_vel_nav 或 /m20_inspection/cmd_vel_manual
  -> cmd_vel_safety_mux_node
  -> /m20_inspection/cmd_vel_safe
  -> motion_adapter_node
  -> /cmd_vel
  -> m20_sdk_deploy/rl_deploy_cmdvel
  -> /JOINTS_CMD
  -> m20_factory_simulation
  -> /JOINTS_DATA + /IMU_DATA
```

其中 `m20_sdk_deploy/rl_deploy_cmdvel` 内部仍然使用官方强化学习策略、状态机和 PD/力矩命令链路；巡检包只看见 `/cmd_vel` 这个稳定接口。

推荐启动入口：

```text
ros2 launch m20_industrial_inspection mujoco_sdk_inspection.launch.py
```

默认启动底层仿真、官方控制器、安全层和 RViz。若底层仿真已经在运行，需要启动任务管理器和 odom 目标跟踪器时：

```text
ros2 launch m20_industrial_inspection mujoco_sdk_inspection.launch.py start_mission:=true
```

当 `start_mission:=true` 时，它只启动任务层，不会重复启动 MuJoCo、官方控制器、RViz 和 robot_state_publisher。如需一条命令同时启动底层和任务，需要显式设置 `force_bringup_with_mission:=true`。

场景文件：

```text
models/mjcf/m20_factory_scene.xml
models/mjcf/official_scene.xml
models/mjcf/official_stair.xml
models/mjcf/M20_factory_robot.xml
```

当前默认场景是之前官方启动场景的项目内副本。`M20_factory_robot.xml` 是从官方 M20 MJCF 复制出的项目内版本，仅调整 mesh 路径以保证安装后能加载。后续如果官方模型更新，需要人工同步这些文件。

`m20_factory_simulation` 的 `enable_drdds_bridge` 参数用于控制是否启用官方低层消息桥接：

```text
enable_drdds_bridge=true
```

启用时需要 `drdds` 消息类型可用，并可与当前官方 RL 控制器通信。

```text
enable_drdds_bridge=false
```

关闭时不再依赖 `drdds`，自定义 MuJoCo 场景仍可启动，但不会接收 `/JOINTS_CMD` 或发布官方控制器需要的低层状态话题。

## 仿真定位接口

项目仿真节点发布 MuJoCo ground-truth 里程计：

```text
/odom
nav_msgs/msg/Odometry
```

默认 TF：

```text
odom -> base_link
```

配置参数：

```text
publish_ground_truth_odom
publish_tf
odom_topic
odom_frame_id
base_frame_id
```

该接口只代表仿真真实位姿，不能等同于真机定位。后续真机巡检时，应由 SLAM、定位算法或传感器融合节点提供 `/odom` 与 TF。

## 命令话题

项目导航速度命令：

```text
/m20_inspection/cmd_vel_nav
geometry_msgs/msg/Twist
```

手动速度命令，预留给键盘、手柄或测试工具：

```text
/m20_inspection/cmd_vel_manual
geometry_msgs/msg/Twist
```

急停信号：

```text
/m20_inspection/e_stop
std_msgs/msg/Bool
```

项目内部安全速度命令：

```text
/m20_inspection/cmd_vel_safe
geometry_msgs/msg/Twist
```

当前仿真后端速度命令：

```text
/cmd_vel
geometry_msgs/msg/Twist
```

该话题由 `m20_sdk_deploy/rl_deploy_cmdvel` 订阅。项目侧参数放在：

```text
config/sdk_deploy_cmdvel.yaml
```

其中配置的是官方包内部节点 `m20_cmd_vel_interface`，不是巡检包自己的节点。

## 巡检任务输出

第一版 `patrol_manager_node` 不直接控制官方后端，只发布项目导航速度命令：

```text
/m20_inspection/cmd_vel_nav
geometry_msgs/msg/Twist
```

当前路线配置是动作序列，不是地图 waypoint：

```text
step_names
step_durations_sec
step_linear_x
step_linear_y
step_angular_z
```

后续接入 Nav2 后，`patrol_manager_node` 应切换为发送导航目标或 action，不应绕过安全层直接发布后端速度。

当前项目主线使用 `inspection_mission_node` 作为上层巡检任务管理器。它不直接发速度，只向目标跟踪器下发目标点：

```text
/m20_inspection/tracker_goal
geometry_msgs/msg/PoseStamped
```

用户侧任务控制输入：

```text
/m20_inspection/mission_control
std_msgs/msg/String
```

支持命令：

```text
pause：暂停任务，并让底层目标跟踪器停止输出速度。
resume：继续执行当前任务。
stop：停止任务，并清空底层当前目标。
restart：从配置里的第一个巡检点重新开始。
```

任务层给底层目标跟踪器的内部控制话题：

```text
/m20_inspection/tracker_control
std_msgs/msg/String
```

正常调试和上层业务不直接发布该话题，避免绕过任务状态机。

它监听目标跟踪器状态：

```text
/m20_inspection/tracker_status
std_msgs/msg/String
```

并发布上层任务状态与 RViz 任务路线：

```text
/m20_inspection/mission_status
std_msgs/msg/String

/m20_inspection/mission_route_marker
visualization_msgs/msg/Marker
```

当前巡检点配置：

```text
config/inspection_mission.yaml
```

RViz 的 `/goal_pose` 在任务演示中先进入 `inspection_mission_node`，再转发到 `/m20_inspection/tracker_goal`。这样可以保证 RViz 临时目标和正式巡检队列都走同一层任务管理逻辑。

## odom 坐标巡检接口

仿真阶段可以使用 `odom_waypoint_patrol_node` 直接追踪 `/odom` 全局坐标点。

输入：

```text
/odom
nav_msgs/msg/Odometry

/goal_pose
geometry_msgs/msg/PoseStamped
```

`/goal_pose` 来自 RViz 的 `2D Goal Pose` 工具，只用于仿真调试阶段临时替换目标点，不作为正式巡检任务接口。

在 `inspection_mission_demo.launch.py` 中，`odom_waypoint_patrol_node` 不直接监听 `/goal_pose`，而是监听：

```text
/m20_inspection/tracker_goal
geometry_msgs/msg/PoseStamped
```

该模式下它只作为目标跟踪器使用，等待上层任务管理器下发目标。

输出：

```text
/m20_inspection/cmd_vel_nav
geometry_msgs/msg/Twist
```

调试输出：

```text
/m20_inspection/patrol_status
std_msgs/msg/String

/m20_inspection/tracker_status
std_msgs/msg/String

/m20_inspection/odom_trace
nav_msgs/msg/Path

/m20_inspection/current_goal
geometry_msgs/msg/PoseStamped

/m20_inspection/current_goal_marker
visualization_msgs/msg/Marker

/m20_inspection/route_marker
visualization_msgs/msg/Marker
```

路线配置：

```text
config/odom_waypoints.yaml
```

该节点不做建图、避障或全局路径规划，只根据当前位姿和目标点生成速度命令。后续接入雷达、地图和 Nav2 后，应由 Nav2 接管路径规划，该节点可作为简单调试工具保留。

当前跟踪器支持两种内部控制模式：

```text
tracking_mode=point_p
```

单点 P 控制：距离误差控制 `linear.x`，朝向误差控制 `angular.z`。

```text
tracking_mode=pure_pursuit
```

简化调节纯跟踪：根据前视点横向偏差计算曲率，接近目标时降速。该模式用于当前 odom 仿真阶段的平滑目标跟踪，不等同于完整 Nav2 Regulated Pure Pursuit。

`/m20_inspection/tracker_status` 或 `/m20_inspection/patrol_status` 中会带有：

```text
tracking_mode=<当前模式>
```

## RViz 机器人模型显示接口

`odom_patrol_demo.launch.py` 默认启动 M20 模型显示链路。该链路只服务于 RViz 可视化，不参与运动控制。

输入：

```text
/JOINTS_DATA
drdds/msg/JointsData
```

桥接输出：

```text
/joint_states
sensor_msgs/msg/JointState
```

模型与 TF：

```text
models/urdf/M20_visual.urdf
  -> robot_state_publisher
  -> /robot_description
  -> base_link 下各关节 TF
```

`M20_visual.urdf` 是从官方 `deep_robotics_model` 的 M20 URDF 生成的项目内副本，只把 mesh 路径改为 `package://m20_industrial_inspection/...`。后续如果官方模型更新，需要人工同步该 URDF。

注意：`/JOINTS_DATA` 不是 URDF 关节坐标，而是 SDK/电机侧坐标。`m20_joint_state_bridge` 默认按当前 M20 仿真和 `M20Interface` 的 `joint_dir + offset` 规则转换后再发布 `/joint_states`。为了让 RViz 初始显示可读，桥接节点在未收到真实关节数据前会发布默认站立姿态，并默认固定四个轮子关节。

RViz 默认配置使用：

```text
Robot Trace: /m20_inspection/odom_trace
Mission: /m20_inspection/mission_route_marker
Tracker Route: /m20_inspection/route_marker
Current Goal Marker: /m20_inspection/current_goal_marker
Current Goal Pose: /m20_inspection/current_goal
```

原始 `/odom` 箭头显示默认关闭，只在需要排查 odom 姿态时手动打开。

## 速度仲裁规则

`cmd_vel_safety_mux_node` 负责生成 `/m20_inspection/cmd_vel_safe`。

优先级：

```text
急停 > 手动控制 > 导航控制 > 零速度
```

如果 `manual_priority=false`，则手动和导航优先级互换：

```text
急停 > 导航控制 > 手动控制 > 零速度
```

安全处理：

- 输入命令超时后输出零速度。
- 限制 `linear.x`、`linear.y`、`angular.z`。
- 清零暂不支持的 `linear.z`、`angular.x`、`angular.y`。
- 按参数限制速度变化率，减少突变指令。

## 后端假设

当前完整 MuJoCo 仿真使用 `src/third_party/sdk_deploy` 中的 `rl_deploy_cmdvel` 桥接节点。
巡检包不应直接依赖官方包内部类，除非该依赖被明确限制在适配层文件内。

## 真机适配预留

如果真机 M20 使用不同接口，只修改运动适配层后端。
巡检逻辑、导航接入、安全限速和巡检任务仍继续使用项目自有话题。

## 安全规则

导航或巡检节点不允许直接发布到官方运动控制话题。
所有运动命令必须先经过本项目安全层。
