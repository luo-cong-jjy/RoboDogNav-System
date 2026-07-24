# Gazebo/Nav2 导航接口约定

## 1. 包职责

`m20_industrial_inspection_gazebo` 负责：

```text
Gazebo 工厂环境
雷达 /scan 或 /LIDAR/POINTS
SLAM / AMCL / Nav2
动态避障
巡检任务点
导航速度命令生成
```

不负责：

```text
官方 sdk_deploy
关节级低层控制
MuJoCo 动力学一致性验证
```

## 2. 单独 Gazebo 验证接口

单独跑 Gazebo/Nav2 时：

```text
Nav2 controller
  -> /cmd_vel
  -> Gazebo 四轮站立代理模型
```

传感器输入：

```text
/LIDAR/POINTS
sensor_msgs/msg/PointCloud2

/scan
sensor_msgs/msg/LaserScan
```

定位和 TF：

```text
/odom
odom -> base_link
```

## 3. 接入 MuJoCo 包时的速度接口

接入 `m20_industrial_inspection_mujoco` 时，Nav2 输出不能直接占用全局 `/cmd_vel`。

应改为：

```text
Nav2 controller /cmd_vel
  --remap-->
/m20_nav2/cmd_vel
```

MuJoCo 包侧会消费：

```text
/m20_nav2/cmd_vel
geometry_msgs/msg/Twist
```

并经过：

```text
/m20_nav2/cmd_vel
  -> /m20_control/cmd_vel_safe
  -> /cmd_vel
  -> m20_sdk_deploy/rl_deploy_cmdvel
```

## 4. 巡检任务接口

巡检任务节点：

```text
factory_inspection_nav2_mission
```

使用 Nav2 action：

```text
navigate_to_pose
nav2_msgs/action/NavigateToPose
```

默认任务点：

```text
config/factory_inspection_midpoints.yaml
```

状态输出：

```text
/m20_factory_inspection/status
/m20_factory_inspection/route_marker
/m20_factory_inspection/current_goal
/m20_factory_inspection/current_goal_marker
```

## 5. 分工原则

```text
Gazebo/Nav2 包：
  负责导航能不能找到路、避开障碍、到达巡检点。

MuJoCo 包：
  负责 M20 接收到速度命令后，官方底层控制能不能稳定、真实地走。
```

两个包的共同接口是速度命令，不共享低层仿真模型。
