# M20 Nav2/Gazebo 传感器验证包

本包用于在实物暂不可用时，先验证工业巡检导航链路：

```text
Gazebo 速腾三维雷达近似仿真
  -> /LIDAR/POINTS
  -> pointcloud_to_laserscan
  -> /scan
  -> SLAM Toolbox / Nav2 costmap
  -> Nav2 planner / controller
  -> /cmd_vel
  -> 单独验证时驱动 Gazebo 导航代理模型
```

它不是 M20 官方动力学仿真的替代品。当前 Gazebo 物理实体是“导航代理模型”或四轮站立形态 M20，用于验证雷达、建图、定位、避障和目标点导航。RViz 显示使用本包内拼接了 `lidar_link` 的 M20 外观模型，避免把完整 16 关节官方模型直接放进 Gazebo 造成不稳定。

后续接入 MuJoCo + 官方 `sdk_deploy` 时，不要让 Nav2 继续直接占用全局 `/cmd_vel`。应把 Nav2 controller 的速度输出 remap 到：

```text
/m20_nav2/cmd_vel
```

再交给 `m20_industrial_inspection_mujoco` 包里的安全仲裁和运动适配层：

```text
/m20_nav2/cmd_vel
  -> /m20_control/cmd_vel_safe
  -> /cmd_vel
  -> m20_sdk_deploy/rl_deploy_cmdvel
```

## 当前已有资产

官方雷达数据包：

```text
datasets/m20/lidar_data_bag_0.db3
```

当前数据包包含：

```text
/LIDAR/POINTS   sensor_msgs/msg/PointCloud2
/IMU            sensor_msgs/msg/Imu
```

已生成的点云地图：

```text
data/office4f/global.pcd
data/office4f/0.pcd
```

这些点云可以作为搭建 Gazebo 场景的参考，但不能直接让 Gazebo 变成可交互的真实世界。Gazebo 里真正可碰撞、可被雷达扫描的墙体和障碍物，需要用 SDF/URDF 模型重建或近似搭建。

## 文件结构

```text
config/nav2_params.yaml              Nav2 参数模板
config/nav2_params_mppi.yaml         MPPI 局部控制器实验参数
config/slam_toolbox.yaml             SLAM Toolbox 参数模板
docs/official_bag_to_gazebo.md       官方 bag 到 Gazebo 验证场景的说明
launch/gazebo_sensor_sandbox.launch.py
launch/gazebo_sensor_m20_factory.launch.py              默认工厂 Gazebo，当前等价 3D 版
launch/gazebo_sensor_m20_factory_3d_rslidar.launch.py   工厂 Gazebo，三维 RoboSense 风格点云
launch/gazebo_sensor_m20_factory_2d_scan.launch.py      工厂 Gazebo，轻量二维 /scan 兼容版
launch/keyboard_teleop.launch.py
launch/nav2_map_navigation_sandbox.launch.py
launch/nav2_map_navigation_m20_factory.launch.py
launch/nav2_map_navigation_mppi_sandbox.launch.py
launch/nav2_slam_sandbox.launch.py
launch/full_sandbox.launch.py
launch/full_m20_factory.launch.py
launch/full_m20_factory_inspection_3d_rslidar.launch.py 巡检，三维点云转 /scan
launch/full_m20_factory_inspection_2d_scan.launch.py    巡检，二维 /scan 兼容版
models/m20_nav_proxy.urdf            平面移动质点代理模型
models/m20_gazebo_combined.urdf      兼容名，保留为当前 3D 雷达模型快照
models/m20_gazebo_combined_rslidar3d.urdf  M20 + 三维 RoboSense 风格点云雷达
models/m20_gazebo_combined_2d_scan.urdf    M20 + 轻量二维 /scan 雷达
models/urdf/M20_nav_visual.urdf      新包内复制的官方 M20 外观模型，已拼接 lidar_link
rviz/nav2_sandbox.rviz               RViz 显示配置
worlds/factory_environment.world
worlds/official_bag_reference_factory.world
```

## 依赖

当前机器尚未安装 Gazebo/Nav2/SLAM Toolbox。后续要运行完整仿真时，ROS2 Humble 下通常需要：

```bash
sudo apt update
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

说明：

- 第一版使用 Gazebo Classic 插件写法，主要为了在 ROS2 Humble 上快速接通 Nav2。
- 后续如果切换到新版 Gazebo，可以迁移到 `ros_gz` 链路。

## 编译

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
colcon build --packages-select m20_nav2_gazebo_sandbox
source install/setup.bash
```

## 启动 Gazebo 传感器仿真

安装 Gazebo 依赖后执行：

```bash
cd /home/virdy/ros2_ysc_ws
conda deactivate  # 如果当前终端还在 Conda 环境中，先退出
source scripts/m20_env.bash
ros2 launch m20_nav2_gazebo_sandbox gazebo_sensor_sandbox.launch.py
```

预期输出：

```text
/LIDAR/POINTS
/scan
/odom
/tf
/robot_description
```

说明：

- Gazebo 里可运动的是平面移动质点代理，不验证真实动力学。
- RViz 里显示的是本包复制的官方 M20 外观、顶部雷达和默认站立关节状态。
- 这样做是为了先稳定验证导航链路，不把问题混进复杂腿部动力学。
- 默认不会在 Gazebo launch 里直接打开 RViz。导航或建图时由 Nav2 launch 延迟启动 RViz，避免 TF 链还没完整时 `/scan` 显示队列堆满。
- 默认会启动 3 个动态障碍物，它们由 Gazebo 内部模型插件平滑移动，不在静态地图中，只靠运行时雷达 `/scan` 进入代价地图。

如果只想单独看 Gazebo 传感器画面，可以临时打开 RViz：

```bash
ros2 launch m20_nav2_gazebo_sandbox gazebo_sensor_sandbox.launch.py use_rviz:=true
```

动态障碍物当前挂载 `libm20_sinusoidal_obstacle_plugin.so`，在 Gazebo 仿真更新回调里按正弦轨迹移动。它不走 `/set_entity_state` 服务，因此不会产生服务请求排队导致的显示延迟。

检查里程计和 TF：

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

## 启动 M20 工厂动力学仿真

当前 `gazebo_sensor_m20_factory.launch.py` 使用：

```text
factory_environment.world
  + m20_gazebo_combined_rslidar3d.urdf
  + libm20_four_wheel_drive_plugin.so
```

这不是官方 MuJoCo RL 步态的等价替代，而是 Gazebo Classic 里的四轮物理底盘：M20 机体和腿部保持站立形态，四个轮关节作为连续关节接触地面并由 `/cmd_vel` 驱动。它适合在 MuJoCo 暂不可用时继续验证工厂场景中的建图、定位、导航、动态避障和巡检流程。

单独启动工厂 Gazebo：

```bash
ros2 launch m20_nav2_gazebo_sandbox gazebo_sensor_m20_factory.launch.py
```

该启动文件会生成硬件话题风格的 `/LIDAR/POINTS` 点云，并默认启动 `pointcloud_to_laserscan` 输出 `/scan`。如果只想检查三维点云本身，可以临时关闭投影：

```bash
ros2 launch m20_nav2_gazebo_sandbox gazebo_sensor_m20_factory.launch.py use_pointcloud_to_scan:=false
```

当前雷达按常见速腾 16 线三维雷达做近似：10 Hz、1800 点/圈、垂直视场约 `-15 deg` 到 `+15 deg`、仿真最大量程 30 m。

如果需要直接看 Gazebo 和三维点云：

```bash
ros2 launch m20_nav2_gazebo_sandbox gazebo_sensor_m20_factory.launch.py use_gazebo_gui:=true use_rviz:=true
```

RViz 配置里已经包含 `/LIDAR/POINTS` 的 `PointCloud2` 显示项。只启动 Gazebo、不启动 Nav2/SLAM 时，如果点云不显示，把 RViz 左上角 `Fixed Frame` 从 `map` 改成 `odom`。

如果需要对比轻量二维雷达旧链路，可以使用：

```bash
ros2 launch m20_nav2_gazebo_sandbox gazebo_sensor_m20_factory_2d_scan.launch.py
```

二维版本加载 `models/m20_gazebo_combined_2d_scan.urdf`，Gazebo 直接发布 `/scan`，不发布 `/LIDAR/POINTS`，也不启动 `pointcloud_to_laserscan`。

同时启动工厂 Gazebo 和 Nav2 地图导航：

```bash
ros2 launch m20_nav2_gazebo_sandbox full_m20_factory.launch.py
```

巡检有两个显式入口：

```bash
ros2 launch m20_nav2_gazebo_sandbox full_m20_factory_inspection_3d_rslidar.launch.py
ros2 launch m20_nav2_gazebo_sandbox full_m20_factory_inspection_2d_scan.launch.py
```

`full_m20_factory_inspection.launch.py` 当前保留为默认入口，等价于三维点云链路。因为此工作区还没有 git commit，仓库里也没有旧备份，无法精确恢复“加三维雷达前”的原始文件；当前拆分前状态已备份在 `backups/20260701_lidar_split_current/`。

如果机器人出生高度需要微调：

```bash
ros2 launch m20_nav2_gazebo_sandbox gazebo_sensor_m20_factory.launch.py z:=0.62
```

如果 `/cmd_vel` 正方向和模型运动方向相反，修改：

```text
models/m20_gazebo_combined.urdf
wheel_velocity_sign
```

默认值为 `-1.0`，对应当前 M20 轮关节轴 `0 -1 0`。

## 键盘遥控建图

启动 Gazebo 后，另开一个终端运行：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_nav2_gazebo_sandbox keyboard_teleop.launch.py
```

如果当前终端环境不允许 launch 子进程读取键盘，可以直接运行可执行程序：

```bash
ros2 run m20_nav2_gazebo_sandbox keyboard_cmd_vel_teleop
```

按键：

```text
w/s : 前进 / 后退
a/d : 左转 / 右转
q/e : 加速 / 减速
x   : 停止
空格: 停止
```

注意：按键只会被当前键盘遥控终端接收，鼠标焦点要放在该终端窗口。

## 启动 Nav2 + SLAM

另开终端执行：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_nav2_gazebo_sandbox nav2_slam_sandbox.launch.py
```

## 加载已保存地图导航

如果已经保存了地图，可以不再启动 SLAM，而是加载静态地图进行定位和导航。

终端 1 启动 Gazebo：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_nav2_gazebo_sandbox gazebo_sensor_sandbox.launch.py
```

终端 2 启动 Nav2 静态地图导航：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_nav2_gazebo_sandbox nav2_map_navigation_sandbox.launch.py
```

默认 `nav2_map_navigation_sandbox.launch.py` 使用 RPP 局部控制器，适合作为稳定基线。

如果要测试动态避障更积极的 MPPI 局部控制器，终端 2 改用：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
ros2 launch m20_nav2_gazebo_sandbox nav2_map_navigation_mppi_sandbox.launch.py
```

对比建议：Gazebo 世界和目标点保持一致，分别启动 RPP 与 MPPI，观察动态障碍物横穿路径时 `/cmd_vel`、`local_costmap` 和机器人绕行幅度。

该命令会在 Nav2 启动后延迟打开 RViz，默认延迟 8 秒。需要调整时：

```bash
ros2 launch m20_nav2_gazebo_sandbox nav2_map_navigation_sandbox.launch.py rviz_delay:=12.0
```

默认加载：

```text
maps/factory_slam_20260608_150509.yaml
```

如果要指定其他地图：

```bash
ros2 launch m20_nav2_gazebo_sandbox nav2_map_navigation_sandbox.launch.py \
  map:=/home/virdy/ros2_ysc_ws/src/m20_nav2_gazebo_sandbox/maps/你的地图.yaml
```

检查定位和地图：

```bash
ros2 topic echo /map --once
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map odom
```

如果定位偏了，在 RViz 里使用 `2D Pose Estimate` 点一下机器人初始位置；然后再用 `2D Goal Pose` 下发目标点。

说明：

- RViz 的 `2D Goal Pose` 当前发送到 `/m20_nav2/goal_pose_raw`。
- 本包会自动启动 `goal_pose_restamper`，再转发到 Nav2 使用的 `/goal_pose`。
- 这样做是为了清掉 RViz 目标点的旧时间戳，避免恢复行为或二次规划时出现 `Extrapolation Error looking up target frame`。
- 当前控制器按“质点导航验证”调参：目标点只要求位置到达，不强求最终朝向；转向过程也不会过早触发进度失败。

也可以同时启动 Gazebo 和 Nav2：

```bash
ros2 launch m20_nav2_gazebo_sandbox full_sandbox.launch.py
```

## 保存 SLAM 地图

确认 `/map` 已经发布：

```bash
ros2 topic echo /map --once
```

保存当前建图结果：

```bash
cd /home/virdy/ros2_ysc_ws
source scripts/m20_env.bash
MAP_NAME=factory_slam_$(date +%Y%m%d_%H%M%S)
ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f /home/virdy/ros2_ysc_ws/src/m20_nav2_gazebo_sandbox/maps/${MAP_NAME} \
  --occ 0.65 \
  --free 0.25 \
  --fmt pgm \
  --mode trinary
```

保存后会生成：

```text
src/m20_nav2_gazebo_sandbox/maps/factory_slam_时间戳.yaml
src/m20_nav2_gazebo_sandbox/maps/factory_slam_时间戳.pgm
```

如果后续要通过包内安装路径加载地图，保存后重新编译一次：

```bash
colcon build --packages-select m20_nav2_gazebo_sandbox
source install/setup.bash
```

## 和 M20 巡检主项目的关系

当前包用于验证高层导航，不直接控制官方 M20 MuJoCo/RL 仿真。

后续成熟后，推荐接入方式是：

```text
Nav2 /cmd_vel
  -> m20_industrial_inspection/cmd_vel_safety_mux_node
  -> m20_industrial_inspection/motion_adapter_node
  -> 官方 rl_deploy_cmdvel 或真机运动接口
```

这样 Gazebo 负责验证雷达导航，MuJoCo/官方程序负责验证 M20 运动控制，两边通过 `/cmd_vel` 和安全仲裁层对齐。

## 动态避障原理

当前静态地图只保存建图时存在的墙体和固定障碍物。后续新出现的人、箱子、推车等动态障碍物，不需要写进地图，而是由运行时传感器实时发现：

```text
/scan
  -> local_costmap obstacle_layer 标记当前障碍
  -> inflation_layer 膨胀安全距离
  -> 局部控制器减速、绕行或触发重规划
```

因此，只要动态障碍物能被雷达扫到，并且位于代价地图的传感器范围内，Nav2 就可以做基础动态避障。这个能力是反应式的：它不会预测障碍物未来轨迹，只会根据当前代价地图选择继续走、减速、绕行、等待或恢复。

当前提供两套局部控制器：

```text
RPP：沿全局路径找前视点，稳定轻量，但动态障碍绕行偏保守。
MPPI：采样多条未来局部轨迹并按障碍、路径、目标等 critic 打分，更适合动态障碍测试。
```

MPPI 仍然不是完整动态障碍预测。它会预测“机器人自己未来几秒的轨迹”，并用当前 costmap 评分；如果要预测人或车未来的位置，需要额外做动态目标跟踪和预测 costmap layer。
