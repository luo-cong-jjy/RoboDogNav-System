# MuJoCo 底层控制接口约定

## 1. 职责边界

`m20_industrial_inspection_mujoco` 只负责：

```text
MuJoCo M20 动力学仿真
官方 m20_sdk_deploy/rl_deploy_cmdvel 控制器启动
速度安全仲裁
项目速度接口到官方 /cmd_vel 的适配
RViz 机器人模型和 /odom 基础显示
```

不负责：

```text
雷达仿真
SLAM / AMCL
Nav2 planner / controller
巡检任务点
waypoint 跟踪
地图和 costmap
```

这些内容由 `m20_industrial_inspection_gazebo` 验证。

## 2. MuJoCo 包内部链路

```text
/m20_nav2/cmd_vel
  -> cmd_vel_safety_mux_node
  -> /m20_control/cmd_vel_safe
  -> motion_adapter_node
  -> /cmd_vel
  -> m20_sdk_deploy/rl_deploy_cmdvel
  -> /JOINTS_CMD
  -> m20_factory_simulation
```

状态回传：

```text
m20_factory_simulation
  -> /JOINTS_DATA
  -> /IMU_DATA
  -> /odom
  -> odom -> base_link TF
```

## 3. 预留给 Gazebo/Nav2 的入口

Gazebo/Nav2 集成时，Nav2 controller 原本会发布：

```text
/cmd_vel
geometry_msgs/msg/Twist
```

但在 MuJoCo + 官方 SDK 链路里，`/cmd_vel` 已经留给官方 `m20_sdk_deploy/rl_deploy_cmdvel` 订阅。

因此集成时必须把 Nav2 的速度输出 remap 到：

```text
/m20_nav2/cmd_vel
geometry_msgs/msg/Twist
```

推荐关系：

```text
Nav2 controller /cmd_vel
  --remap-->
/m20_nav2/cmd_vel
  -> /m20_control/cmd_vel_safe
  -> /cmd_vel
  -> sdk_deploy
```

这样可以保留 Gazebo/Nav2 包里的导航验证成果，同时不破坏官方底层控制器的 `/cmd_vel` 接口。

## 4. 安全控制接口

上层速度输入：

```text
/m20_nav2/cmd_vel
geometry_msgs/msg/Twist
```

手动速度输入：

```text
/m20_control/cmd_vel_manual
geometry_msgs/msg/Twist
```

急停：

```text
/m20_control/e_stop
std_msgs/msg/Bool
```

安全层输出：

```text
/m20_control/cmd_vel_safe
geometry_msgs/msg/Twist
```

官方控制器输入：

```text
/cmd_vel
geometry_msgs/msg/Twist
```

## 5. 官方低层接口

官方 `m20_sdk_deploy/rl_deploy_cmdvel` 消费：

```text
/cmd_vel
```

并发布：

```text
/JOINTS_CMD
drdds/msg/JointsDataCmd
```

MuJoCo 仿真节点消费：

```text
/JOINTS_CMD
```

并发布：

```text
/JOINTS_DATA
drdds/msg/JointsData

/IMU_DATA
drdds/msg/ImuData
```

## 6. 仿真定位

MuJoCo 节点发布 ground truth：

```text
/odom
nav_msgs/msg/Odometry
```

以及 TF：

```text
odom -> base_link
```

该 `/odom` 只用于底层控制效果观察和后续集成调试，不代表真实机器人定位系统。

## 7. 启动入口

完整底层链路：

```bash
ros2 launch m20_industrial_inspection_mujoco mujoco_sdk_bringup.launch.py
```

只启动 MuJoCo 仿真：

```bash
ros2 launch m20_industrial_inspection_mujoco factory_sim.launch.py
```

只启动安全层和运动适配：

```bash
ros2 launch m20_industrial_inspection_mujoco control_core.launch.py
```

兼容旧名字：

```bash
ros2 launch m20_industrial_inspection_mujoco mujoco_sdk_inspection.launch.py
```

该兼容入口现在只转到 `mujoco_sdk_bringup.launch.py`，不再包含任务层。

## 8. Gazebo/Nav2 包的当前角色

`m20_industrial_inspection_gazebo` 当前负责：

```text
Gazebo 工厂场景
2D / 3D 雷达仿真
/scan 与 /LIDAR/POINTS
SLAM Toolbox / AMCL / Nav2
巡检任务 action
动态障碍物和避障
```

两边最终的对齐点是速度命令接口，而不是把两套仿真器混成一个进程。
