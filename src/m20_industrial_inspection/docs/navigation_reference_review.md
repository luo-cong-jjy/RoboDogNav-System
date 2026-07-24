# 导航参考项目评估

本文记录当前巡检项目可参考的开源导航资源，以及这些资源和山猫 M20 项目的适配关系。

## 已拉取的参考仓库

参考仓库统一放在工作空间的 `references/navigation_projects` 下，并已添加 `COLCON_IGNORE`，不会参与当前 `colcon build`。

```text
references/navigation_projects/navigation2_humble
references/navigation_projects/rosnav
```

注意：

- 这些仓库当前只作为阅读和参考资料。
- 不建议直接把它们放进 `src` 编译，避免依赖冲突和构建时间暴涨。
- 后续真正使用 Nav2 时，优先通过系统包安装 Nav2，再把参数、launch 和接口接入本项目。

## Navigation2

仓库：

```text
references/navigation_projects/navigation2_humble
```

参考价值最高。它是 ROS2 里标准的导航框架，后续 M20 真机导航也应该围绕它做接口适配。

重点参考模块：

- `nav2_bringup`：Nav2 标准启动方式和参数组织。
- `nav2_costmap_2d`：全局/局部代价地图、障碍物层、膨胀层。
- `nav2_map_server`：加载和保存 2D 栅格地图。
- `nav2_amcl`：基于已有地图和激光的 2D 定位。
- `nav2_controller`：局部控制器框架。
- `nav2_regulated_pure_pursuit_controller`：带速度调节和碰撞检查的纯追踪控制器。
- `nav2_dwb_controller`：可配置的局部轨迹采样控制器，适合需要横向速度能力的平台参考。
- `nav2_mppi_controller`：采样优化控制器，复杂但能力更强。
- `nav2_collision_monitor`：基于传感器区域的急停、减速和避障保护。
- `nav2_waypoint_follower`：多巡检点执行逻辑。
- `nav2_simple_commander`：Python 侧下发导航任务的简洁接口。

对 M20 的判断：

- 当前项目已经实现了简化版 waypoint 跟踪和安全仲裁，后续可以逐步替换为 Nav2 的规划、控制和 costmap。
- M20 是四足轮足/足式平台，不是普通差速小车。Nav2 只负责高层导航，底层仍应由官方运动控制器或我们自己的运动适配层执行。
- 如果只使用 `cmd_vel` 接口，Nav2 可以把 M20 当作移动底盘；但机器人真实可行速度、转弯半径、楼梯能力和急停策略必须由本项目单独约束。

## rosnav

仓库：

```text
references/navigation_projects/rosnav
```

这是一个更完整的 ROS2 导航工程范例，包含 Gazebo、SLAM Toolbox、Nav2、RViz、巡检点、碰撞监控和任务脚本。

可重点参考：

- `src/diff_drive_robot-main/config/nav2_params.yaml`：完整 Nav2 参数组织。
- `src/diff_drive_robot-main/config/costmap.yaml`：代价地图、障碍物层、膨胀层配置。
- `src/diff_drive_robot-main/config/slam_params.yaml`：SLAM Toolbox 参数。
- `src/diff_drive_robot-main/launch/nav2.launch.py`：Nav2 启动结构。
- `src/diff_drive_robot-main/launch/slam_nav.launch.py`：SLAM + Nav2 组合启动方式。
- `src/diff_drive_robot-main/rviz/bot.rviz`：较完整的 RViz 显示配置。
- `src/diff_drive_robot-main/urdf/lidar.xacro`：2D 激光雷达仿真写法。
- `src/diff_drive_robot-main/urdf/lidar3d.xacro`：3D 激光雷达仿真写法。
- `src/diff_drive_robot-main/worlds/warehouse.world`：仓库/工厂类场景参考。
- `src/diff_drive_robot-main/scripts/waypoint_nav.py`：巡检点导航脚本参考。
- `src/diff_drive_robot-main/scripts/collision_monitor.py`：简化碰撞监控逻辑参考。

不能直接照搬的地方：

- rosnav 的机器人是差速轮式底盘，不是 M20。
- 它的 URDF、控制器、里程计和传感器接口不能直接当作 M20 真机接口。
- 它使用 Gazebo 仿真链路，和当前官方 M20 MuJoCo 仿真不是同一个动力学后端。

对 M20 的参考方式：

- 参考它如何组织 `nav2_params.yaml`、costmap、SLAM、RViz 和 waypoint。
- 不直接复用它的底盘控制、URDF 结构和差速运动模型。
- 如果我们要先做雷达避障仿真，可以参考它的 Gazebo world 和 lidar xacro，单独做一个“导航传感器验证仿真”，不要塞进当前 MuJoCo 控制链路里。

## Nav2 传感器、建图和定位文档

官方文档地址：

```text
https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
```

它适合指导后续真机阶段的传感器接入顺序：

1. 先确认传感器话题，例如 2D `/scan` 或 3D `/points`。
2. 建图阶段使用 SLAM 输出 `map -> odom` 或地图结果。
3. 定位阶段使用已有地图和传感器数据，输出稳定的 `map -> odom`。
4. Nav2 使用 `map -> odom -> base_link` 的 TF 链路进行规划和控制。

当前 M20 项目的关键问题：

- 当前 MuJoCo 仿真没有可用的雷达传感器话题。
- 当前 `/odom` 是仿真真值，只适合做巡检控制原型，不等于真实定位。
- 官方 lightning 例程使用真实雷达数据包做建图，不是从当前 MuJoCo 场景里仿真出雷达。

## 推荐路线

### 阶段 1：保留当前 MuJoCo/Odom 巡检原型

目标：

- 保持机器狗运动链路稳定。
- 继续验证任务层、限速、安全仲裁、RViz 目标点和巡检状态。
- 不在没有雷达的 MuJoCo 场景里强行做“假避障”。

当前链路：

```text
inspection_mission_node
  -> odom_waypoint_patrol_node
  -> cmd_vel_safety_mux_node
  -> motion_adapter_node
  -> /cmd_vel
  -> 官方 rl_deploy_cmdvel
```

### 阶段 2：单独建立 Nav2/Gazebo 传感器验证仿真

目标：

- 用 Gazebo 或其他支持传感器的仿真环境搭建简化 M20 外形。
- 先不追求真实动力学，只验证 `scan/points -> costmap -> Nav2 -> cmd_vel`。
- 参考 rosnav 的 world、lidar、nav2_params 和 rviz 配置。

建议链路：

```text
Gazebo 仿真雷达
  -> /scan 或 /points
  -> slam_toolbox / amcl
  -> Nav2 costmap
  -> Nav2 planner/controller
  -> /cmd_vel
  -> 本项目 motion_adapter_node
```

当前已创建独立验证包：

```text
src/m20_nav2_gazebo_sandbox
```

该包只用于传感器和导航链路验证，不替代官方 MuJoCo/RL 运动控制。

### 阶段 3：真机传感器接入

目标：

- 实机确认雷达、IMU、里程计、TF 的真实话题。
- 用 rosbag 先离线分析，不直接让机器狗高速自主跑。
- 将真实定位输出接入 `map -> odom -> base_link`。
- 把 Nav2 输出的 `/cmd_vel` 接入本项目安全仲裁层。

安全原则：

- 真机第一次测试只允许低速。
- 必须保留急停和手动接管。
- 先录包、回放、仿真验证，再闭环上机。

## 当前结论

可以参考，而且已经拉取。

最值得采用的是 Nav2 的标准架构；rosnav 适合作为工程组织和仿真传感器样例。下一步不建议马上把 Nav2 接入当前 MuJoCo，因为当前 MuJoCo 没有雷达输入。更稳的做法是先做一个独立的 Nav2/Gazebo 传感器验证分支，验证避障和建图逻辑，再把成熟接口迁回 M20 巡检项目。
