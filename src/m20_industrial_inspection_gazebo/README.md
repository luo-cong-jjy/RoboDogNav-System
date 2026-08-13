# M20 Gazebo/Nav2 导航避障包

## 1. 包定位

`m20_industrial_inspection_gazebo` 是 M20 的 Gazebo/Nav2 专用包，负责验证：

```text
Gazebo 工厂场景
2D / 3D 雷达仿真
/scan 与 /LIDAR/POINTS
SLAM Toolbox / AMCL
Nav2 地图导航
动态障碍物避障
工厂巡检任务点
```

本包不负责官方低层关节控制、强化学习步态或真实动力学一致性验证。这些内容由：

```text
m20_industrial_inspection_mujoco
```

负责。

## 2. 当前导航链路

```text
Gazebo 雷达
  -> /LIDAR/POINTS
  -> pointcloud_to_laserscan
  -> /scan
  -> SLAM Toolbox / Nav2 costmap
  -> Nav2 planner / controller
  -> /cmd_vel
  -> Gazebo 导航代理模型
```

当前 Gazebo 里的 M20 是导航代理模型或四轮站立形态模型，用来验证导航、避障和巡检流程，不用来评估腿轮真实动力学。

## 3. 预留给 MuJoCo 包的接口

单独运行 Gazebo/Nav2 时，本包可以继续使用 `/cmd_vel` 驱动 Gazebo 代理模型。

后续把 Nav2 导航结果接到 `m20_industrial_inspection_mujoco` 时，不要让 Nav2 直接占用全局 `/cmd_vel`。应把 Nav2 controller 输出 remap 到：

```text
/m20_nav2/cmd_vel
```

再交给 MuJoCo 包的安全层：

```text
/m20_nav2/cmd_vel
  -> /m20_control/cmd_vel_safe
  -> /cmd_vel
  -> m20_sdk_deploy/rl_deploy_cmdvel
```

这样 `/cmd_vel` 仍然只留给官方底层控制器使用，Nav2 不会绕过安全限速和急停。

## 4. 编译

```bash
cd /home/virdyn/robodog_nav_system
source /opt/ros/humble/setup.bash
colcon build --packages-select m20_industrial_inspection_gazebo
source install/setup.bash
```

Gazebo/Nav2 运行依赖通常包括：

```bash
sudo apt install \
  gazebo \
  libgazebo-dev \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-slam-toolbox \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins
```

## 5. 主要文件

```text
config/nav2_params.yaml
config/nav2_params_mppi.yaml
config/slam_toolbox.yaml
config/factory_inspection_midpoints.yaml

launch/gazebo_sensor_m20_factory.launch.py
launch/gazebo_sensor_m20_factory_3d_rslidar.launch.py
launch/gazebo_sensor_m20_factory_2d_scan.launch.py
launch/nav2_map_navigation_m20_factory.launch.py
launch/nav2_slam_sandbox.launch.py
launch/full_m20_factory.launch.py
launch/full_m20_factory_inspection_3d_rslidar.launch.py
launch/full_m20_factory_inspection_2d_scan.launch.py

models/m20_gazebo_combined_rslidar3d.urdf
models/m20_gazebo_combined_2d_scan.urdf
models/urdf/M20_nav_visual.urdf
worlds/factory_environment_v3_dynamic_obstacles.world
rviz/nav2_sandbox.rviz
```

## 6. 启动 Gazebo 雷达仿真

```bash
ros2 launch m20_industrial_inspection_gazebo gazebo_sensor_m20_factory.launch.py
```

常见输出：

```text
/LIDAR/POINTS
/scan
/odom
/tf
/robot_description
```

只看 Gazebo 和 RViz：

```bash
ros2 launch m20_industrial_inspection_gazebo gazebo_sensor_m20_factory.launch.py \
  use_gazebo_gui:=true use_rviz:=true
```

使用轻量二维 `/scan` 模型：

```bash
ros2 launch m20_industrial_inspection_gazebo gazebo_sensor_m20_factory_2d_scan.launch.py
```

## 7. 启动 Nav2 地图导航

先启动 Gazebo 工厂场景后，再启动 Nav2：

```bash
ros2 launch m20_industrial_inspection_gazebo nav2_map_navigation_m20_factory.launch.py
```

也可以使用组合入口：

```bash
ros2 launch m20_industrial_inspection_gazebo full_m20_factory.launch.py
```

## 8. 启动巡检任务

三维雷达链路：

```bash
ros2 launch m20_industrial_inspection_gazebo full_m20_factory_inspection_3d_rslidar.launch.py
```

二维 `/scan` 链路：

```bash
ros2 launch m20_industrial_inspection_gazebo full_m20_factory_inspection_2d_scan.launch.py
```

巡检任务点配置：

```text
config/factory_inspection_midpoints.yaml
```

## 9. 和 MuJoCo 包的分工

```text
m20_industrial_inspection_gazebo:
  负责看路、建图、避障、规划路径、生成速度命令。

m20_industrial_inspection_mujoco:
  负责 M20 是否像真实机器狗一样运动，以及官方底层控制是否稳定。
```

两个包最终通过速度接口对齐，不把两套仿真器混在同一个包里。
