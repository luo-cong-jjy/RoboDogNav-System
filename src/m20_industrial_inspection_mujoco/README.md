# M20 MuJoCo 底层控制验证包

## 1. 包定位

`m20_industrial_inspection_mujoco` 只负责验证：

```text
M20 MuJoCo 动力学
  + 官方 sdk_deploy 强化学习控制器
  + /cmd_vel 到真实关节控制效果
  + 安全限速和急停保护
```

本包不再放 waypoint 巡检、任务管理、SLAM、Nav2、雷达仿真等导航逻辑。

导航、建图、雷达、避障和巡检任务由：

```text
src/m20_industrial_inspection_gazebo
```

负责验证。

## 2. 当前控制链路

```text
外部上层速度输入
  -> /m20_nav2/cmd_vel
  -> cmd_vel_safety_mux_node
  -> /m20_control/cmd_vel_safe
  -> motion_adapter_node
  -> /cmd_vel
  -> m20_sdk_deploy/rl_deploy_cmdvel
  -> /JOINTS_CMD
  -> m20_factory_simulation
  -> /JOINTS_DATA + /IMU_DATA + /odom
```

关键点：

1. `/cmd_vel` 留给官方 `m20_sdk_deploy/rl_deploy_cmdvel` 使用。
2. Gazebo/Nav2 将来接入时，不要直接发布到 `/cmd_vel`。
3. Nav2 的 controller 输出应 remap 到 `/m20_nav2/cmd_vel`，再进入本包安全层。

## 3. 编译

```bash
cd /home/virdyn/robodog_nav_system
source /opt/ros/humble/setup.bash
colcon build --packages-select m20_industrial_inspection_mujoco
source install/setup.bash
```

## 4. 启动 MuJoCo + 官方底层控制

```bash
ros2 launch m20_industrial_inspection_mujoco mujoco_sdk_bringup.launch.py
```

默认启动：

```text
m20_factory_simulation                 # MuJoCo 16 自由度 M20 仿真
m20_sdk_deploy/rl_deploy_cmdvel        # 官方底层控制器
cmd_vel_safety_mux_node                # 安全限速、急停、手动优先
motion_adapter_node                    # 安全速度到官方 /cmd_vel 的适配
m20_joint_state_bridge                 # RViz 关节显示桥
robot_state_publisher
rviz2
```

只跑底层，不打开 RViz：

```bash
ros2 launch m20_industrial_inspection_mujoco mujoco_sdk_bringup.launch.py \
  use_rviz:=false use_robot_model:=false
```

只启动安全层和运动适配：

```bash
ros2 launch m20_industrial_inspection_mujoco control_core.launch.py
```

## 5. 手动测试速度输入

向预留的上层接口发速度：

```bash
ros2 topic pub /m20_nav2/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

急停：

```bash
ros2 topic pub --once /m20_control/e_stop std_msgs/msg/Bool "{data: true}"
```

解除急停：

```bash
ros2 topic pub --once /m20_control/e_stop std_msgs/msg/Bool "{data: false}"
```

## 6. Gazebo/Nav2 集成预留接口

Gazebo/Nav2 包当前用于验证：

```text
雷达 /scan 或 /LIDAR/POINTS
SLAM / AMCL
Nav2 planner/controller
巡检任务点
动态避障
```

等要把 Gazebo/Nav2 的导航结果接入 MuJoCo/官方底层控制时，应保持下面的接口关系：

```text
Nav2 controller 原始输出 /cmd_vel
  --remap-->
/m20_nav2/cmd_vel
  -> MuJoCo 包安全层
  -> /m20_control/cmd_vel_safe
  -> /cmd_vel
  -> 官方 sdk_deploy
```

这样做可以避免 Nav2 和官方 `rl_deploy_cmdvel` 同时使用 `/cmd_vel` 造成抢话题。

详细接口约定见：

```text
docs/interface_contract.md
```

## 7. 文件说明

```text
config/factory_sim.yaml          MuJoCo 场景、初始位姿、odom/TF 配置
config/sdk_deploy_cmdvel.yaml    官方 m20_sdk_deploy/rl_deploy_cmdvel 参数
config/safety.yaml               上层速度输入、安全限速、急停、手动优先
config/sim.yaml                  motion_adapter_node 到官方 /cmd_vel 的适配
launch/mujoco_sdk_bringup.launch.py
launch/control_core.launch.py
launch/factory_sim.launch.py
rviz/mujoco_control.rviz
models/mjcf
models/urdf/M20_visual.urdf
```

## 8. 已移出的内容

以下能力不再属于本 MuJoCo 包：

```text
waypoint 巡检
巡检任务状态机
RViz 目标点输入
Nav2 action 任务
雷达 /scan 或点云仿真
SLAM / AMCL / costmap
```

这些能力统一放在 `m20_industrial_inspection_gazebo` 方向继续发展。`m20_nav2_gazebo_sandbox` 只作为早期参考和备份保留。
