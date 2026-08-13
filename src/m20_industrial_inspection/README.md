# M20 工业巡检项目

本包是山猫 M20 工业巡检机器狗的独立项目包。
它只通过明确的 ROS 接口依赖官方例程，避免巡检业务逻辑直接绑定官方示例内部实现。

## 项目范围

当前目标：

- 使用已下载的官方 M20 仿真和训练好的运动策略。
- 建立本项目自己的巡检、限速、安全仲裁和运动适配链路。
- 为真机接口预留清晰边界，避免默认假设真机程序一定和当前例程一致。

第一阶段暂不做：

- 不直接修改 `sdk_deploy`、`deep_robotics_model`、`lightning-lm-deep-robotics`。
- 不假设真机接口一定等同于当前仿真接口。

## 规划架构

```text
巡检任务 / 导航
  -> /m20_inspection/cmd_vel_nav
  -> 安全仲裁与限速
  -> /m20_inspection/cmd_vel_safe
  -> 运动适配层
  -> /cmd_vel
  -> 官方 m20_sdk_deploy/rl_deploy_cmdvel
  -> /JOINTS_CMD
  -> MuJoCo 仿真或真机 SDK
```

核心隔离点是运动适配层。巡检包不直接链接或复制 `sdk_deploy` 代码；仿真阶段它只把速度命令发布到 `/cmd_vel`，由第三方包里的 `m20_sdk_deploy/rl_deploy_cmdvel` 接入官方策略。真机阶段如果接口不同，只替换适配层后端或替换 `/cmd_vel` 后面的控制提供者。

## 规划节点

- `cmd_vel_safety_mux_node`：接收导航/手动命令，处理超时、限速、加速度限制和急停。
- `motion_adapter_node`：把项目内部安全速度命令转换为当前后端需要的命令接口。
- `patrol_manager_node`：加载巡检动作路线，后续接入导航 action 和巡检任务。
- `odom_waypoint_patrol_node`：仿真阶段直接使用 `/odom` 全局坐标进行 waypoint 巡检。
- `inspection_mission_node`：上层巡检任务管理器，负责下发巡检点、监听跟踪状态，并在 RViz 中显示任务路线。

## 关键接口

见 `docs/interface_contract.md`。

导航参考项目评估见：

```text
docs/navigation_reference_review.md
```

Gazebo/Nav2 传感器验证包位于：

```text
../m20_nav2_gazebo_sandbox
```

## 编译

在工作空间根目录执行：

```bash
cd /home/virdy/ros2_ysc_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select m20_industrial_inspection
source install/setup.bash
```

## 当前运行

启动项目自有仿真入口：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_industrial_inspection factory_sim.launch.py
```

启动完整联调链路：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_industrial_inspection mujoco_sdk_inspection.launch.py
```

默认不会自动启动巡检任务层，避免仿真刚启动就运动。若底层仿真已经在运行，需要启动巡检任务层时：

```bash
ros2 launch m20_industrial_inspection mujoco_sdk_inspection.launch.py start_mission:=true
```

现在这条命令只启动任务层，不会再重复启动 MuJoCo、官方控制器、RViz 和 robot_state_publisher。
如果确实要一条命令同时启动底层和任务，需要额外设置 `force_bringup_with_mission:=true`。

这条链路把各模块分开启动：

```text
m20_industrial_inspection:
  cmd_vel_safety_mux_node
  motion_adapter_node
  m20_factory_simulation
  odom_waypoint_patrol_node        # start_mission:=true 时启动
  inspection_mission_node          # start_mission:=true 时启动

third_party/sdk_deploy:
  m20_sdk_deploy/rl_deploy_cmdvel
  drdds
```

话题连接关系：

```text
/m20_inspection/cmd_vel_nav
  -> /m20_inspection/cmd_vel_safe
  -> /cmd_vel
  -> m20_sdk_deploy/rl_deploy_cmdvel
  -> /JOINTS_CMD
  -> m20_factory_simulation
  -> /JOINTS_DATA + /IMU_DATA + /odom
```

也可以使用快捷脚本：

```bash
/home/virdy/ros2_ysc_ws/scripts/run_m20_factory_sim.bash
```

自定义场景位于：

```text
src/m20_industrial_inspection/models/mjcf/m20_factory_scene.xml
```

当前默认场景已经切换为官方 M20 场景的项目内副本：

```text
src/m20_industrial_inspection/models/mjcf/official_scene.xml
src/m20_industrial_inspection/models/mjcf/official_stair.xml
```

它对应之前官方启动使用的组合：

```text
M20.xml + scene.xml + stair.xml
```

机器人初始位姿也改回官方默认：

```text
位置：(0.0, 0.0, 0.2)
朝向：yaw=0.0
```

早期创建的简化工厂场景仍保留为备用：

```text
src/m20_industrial_inspection/models/mjcf/factory_world.xml
```

## 与官方包的独立性

当前仿真的官方场景副本和 M20 mesh 已经放在本项目内：

```text
src/m20_industrial_inspection/models
```

因此仅打开自定义 MuJoCo 场景时，不再依赖官方 `sdk_deploy` 的模型目录。

如果没有官方 `drdds` 消息包，也可以关闭低层控制桥接，只启动场景：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 run m20_industrial_inspection m20_factory_simulation \
  --ros-args -p enable_drdds_bridge:=false
```

注意：

- 关闭 `enable_drdds_bridge` 后，只能运行自定义 MuJoCo 场景，不会和官方 RL 控制器通信。
- 如果要继续使用当前 `rl_deploy_cmdvel` 控制机器狗，仍需要官方控制器和 `drdds` 消息类型可用。
- 后续如果要彻底摆脱官方控制器，需要我们自己实现低层控制后端或重新定义控制消息接口。

仿真节点会发布导航基础信息：

```text
/odom
odom -> base_link
```

这是 MuJoCo ground truth，用于仿真阶段验证导航、巡检和避障链路。真机阶段不能直接照搬，需要替换为真实定位、SLAM 或传感器融合输出。

只启动运动适配层：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_industrial_inspection motion_adapter.launch.py
```

它会把 `/m20_inspection/cmd_vel_safe` 转发到当前官方控制后端 `/cmd_vel`。

启动安全仲裁和运动适配层：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_industrial_inspection inspection_core.launch.py
```

测试导航速度输入：

```bash
ros2 topic pub /m20_inspection/cmd_vel_nav geometry_msgs/msg/Twist \
"{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

触发急停：

```bash
ros2 topic pub --once /m20_inspection/e_stop std_msgs/msg/Bool "{data: true}"
```

解除急停：

```bash
ros2 topic pub --once /m20_inspection/e_stop std_msgs/msg/Bool "{data: false}"
```

当前链路：

```text
/m20_inspection/cmd_vel_nav 或 /m20_inspection/cmd_vel_manual
  -> cmd_vel_safety_mux_node
  -> /m20_inspection/cmd_vel_safe
  -> motion_adapter_node
  -> /cmd_vel
  -> m20_sdk_deploy/rl_deploy_cmdvel
  -> /JOINTS_CMD
```

启动最小巡检演示：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_industrial_inspection patrol_demo.launch.py
```

第一版巡检演示读取 `config/patrol_routes.yaml`，按动作序列发布速度：

```text
前进 -> 停止 -> 原地左转 -> 停止
```

它还不是基于地图的自主导航，只用于验证：

```text
巡检任务层 -> 安全仲裁层 -> 运动适配层 -> 官方运动控制
```

启动 odom 全局坐标巡检演示：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_industrial_inspection odom_patrol_demo.launch.py
```

该演示默认会同时打开 RViz。若只想启动节点，不打开 RViz：

```bash
ros2 launch m20_industrial_inspection odom_patrol_demo.launch.py use_rviz:=false
```

该演示读取 `config/odom_waypoints.yaml`，直接使用 `/odom` 里的仿真全局坐标追踪 waypoint。

```text
/odom
  -> odom_waypoint_patrol_node
  -> /m20_inspection/cmd_vel_nav
  -> 安全仲裁与运动适配
  -> /cmd_vel
```

这不是 SLAM，也不是 Nav2。它只用于当前没有雷达仿真的阶段，先验证巡检控制链路和坐标逻辑。

`odom_waypoint_patrol_node` 当前支持两种跟踪模式：

```text
point_p：旧版单点 P 控制，距离误差控制前进速度，朝向误差控制转向。
pure_pursuit：简化调节纯跟踪，按前视点计算曲率，并在接近目标时自动降速。
```

当前默认使用 `pure_pursuit`。它还不是完整 Nav2 Regulated Pure Pursuit，只是为当前 odom 仿真阶段提供更平滑、可调的目标跟踪器。

主要参数在 `config/odom_tracker.yaml` 和 `config/odom_waypoints.yaml` 中：

```yaml
tracking_mode: pure_pursuit
goal_tolerance_m: 0.08
max_linear_speed: 0.35
max_angular_speed: 0.55
pure_pursuit_lookahead_distance: 0.45
pure_pursuit_min_linear_speed: 0.03
pure_pursuit_slowdown_distance: 0.75
pure_pursuit_curvature_gain: 1.0
```

`goal_tolerance_m` 决定“离目标点多近算到达”。如果设置过大，RViz 中会看到机器人几何中心没有和目标点完全重合；如果设置过小，当前无避障、无精确定位的仿真阶段可能会在目标附近来回修正。当前 0.08m 是速度和到点精度之间的折中。

如果需要临时退回旧控制方式：

```yaml
tracking_mode: point_p
```

启动上层巡检任务演示：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_industrial_inspection inspection_mission_demo.launch.py
```

这个入口是后续项目主线。它把任务层和跟踪层拆开：

```text
inspection_mission_node
  -> /m20_inspection/tracker_goal
  -> odom_waypoint_patrol_node
  -> /m20_inspection/cmd_vel_nav
  -> 安全仲裁与运动适配
  -> /cmd_vel
```

当前默认只保留一个巡检点：

```text
src/m20_industrial_inspection/config/inspection_mission.yaml
```

RViz 的 `2D Goal Pose` 会先进入 `inspection_mission_node`，再由它转发给 odom 跟踪器。这样以后接 Nav2、雷达避障或 NavRL 风格局部规划时，只替换任务层后面的执行器，不需要改 RViz 操作习惯。

RViz 配置参考了 NavRL ROS2 示例的显示组织方式，当前重点显示：

```text
Robot：M20 模型
Robot Trace：机器人运动轨迹线
Mission：上层巡检任务点、路线和文字标签
Tracker Route：底层目标跟踪器路线
Current Goal Marker：当前执行目标点
```

原始 `/odom` 箭头默认关闭，因为箭头密集时不利于观察巡检路线。需要调试原始 odom 时，在 RViz Displays 中手动打开 `Raw Odom Pose`。

调试话题：

```text
/m20_inspection/patrol_status
/m20_inspection/tracker_status
/m20_inspection/mission_status
/m20_inspection/odom_trace
/m20_inspection/current_goal
/m20_inspection/current_goal_marker
/m20_inspection/route_marker
/m20_inspection/mission_route_marker
```

查看当前巡检状态：

```bash
ros2 topic echo /m20_inspection/patrol_status --no-daemon
```

查看上层任务状态：

```bash
ros2 topic echo /m20_inspection/mission_status --no-daemon
```

控制巡检任务：

```bash
# 暂停：任务层保持当前状态，底层跟踪器停止输出速度。
ros2 topic pub --once /m20_inspection/mission_control std_msgs/msg/String "{data: pause}"

# 继续：从暂停点继续执行当前目标。
ros2 topic pub --once /m20_inspection/mission_control std_msgs/msg/String "{data: resume}"

# 停止：清空底层当前目标，任务不再自动继续。
ros2 topic pub --once /m20_inspection/mission_control std_msgs/msg/String "{data: stop}"

# 重启：从配置里的第一个巡检点重新开始。
ros2 topic pub --once /m20_inspection/mission_control std_msgs/msg/String "{data: restart}"
```

用户侧只需要使用 `/m20_inspection/mission_control`。`/m20_inspection/tracker_control` 是任务层给底层目标跟踪器的内部控制话题，正常调试时不用直接发布。

在 RViz 中可添加：

```text
RobotModel: /robot_description
Path: /m20_inspection/odom_trace
Pose: /m20_inspection/current_goal
Marker: /m20_inspection/current_goal_marker
Marker: /m20_inspection/route_marker
Marker: /m20_inspection/mission_route_marker
```

`odom_patrol_demo.launch.py` 默认已经启动 M20 模型显示链路：

```text
/JOINTS_DATA
  -> m20_joint_state_bridge
  -> /joint_states
  -> robot_state_publisher
  -> /robot_description + TF
  -> RViz RobotModel
```

`/JOINTS_DATA` 是 SDK/电机侧关节坐标，`m20_joint_state_bridge` 会转换为 URDF 关节坐标后再发布 `/joint_states`。为方便观察初始化状态，桥接节点在没有收到真实关节数据前会先发布一组站立静止姿态，并默认固定四个轮子关节。

RViz 中如果出现 `TF_OLD_DATA ignoring data from the past`，通常是上游关节状态或 odom TF 使用了从 0 开始的仿真时间戳，而 RViz 使用系统当前时间。当前项目仿真默认 `use_ros_time_stamp: true`，关节桥接默认 `use_current_time_stamp: true`，用于保证 RViz 可视化时间戳一致。

如果需要在 RViz 中显示轮子转动，可以关闭轮子固定：

```bash
ros2 launch m20_industrial_inspection odom_patrol_demo.launch.py freeze_wheel_joints:=false
```

轮子关节是连续旋转关节，显示效果可能会比较跳，当前主要还是以机身和腿部姿态可读为准。

如果第二个终端的官方控制程序没有启动，RViz 中的模型会保持默认站立姿态或不再更新。只想看导航点和 odom 时，可以关闭模型链路：

```bash
ros2 launch m20_industrial_inspection odom_patrol_demo.launch.py use_robot_model:=false
```

RViz 启动后可使用工具栏的 `2D Goal Pose` 临时下发单个目标点。节点会把路线临时替换为这个目标点，便于快速试点位；如果要永久修改巡检路线，编辑：

```text
src/m20_industrial_inspection/config/odom_waypoints.yaml
```

主要改这两行：

```yaml
waypoint_x: [0.0]
waypoint_y: [0.8]
```

改完配置后重新编译并启动：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
colcon build --packages-select m20_industrial_inspection
source scripts/m20_env.bash
ros2 launch m20_industrial_inspection odom_patrol_demo.launch.py
```

配合自定义仿真运行时，推荐三个终端：

终端 1：

```bash
/home/virdy/ros2_ysc_ws/scripts/run_m20_factory_sim.bash
```

终端 2：

```bash
/home/virdy/ros2_ysc_ws/scripts/run_m20_rl_deploy_cmdvel.bash
```

终端 3：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_industrial_inspection inspection_mission_demo.launch.py
```
