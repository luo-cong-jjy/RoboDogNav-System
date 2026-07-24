# M20 Foxy Nav2 Deployment

这个包是给 M20 背部搭载 x86 主机使用的 ROS2 Foxy 导航部署模板。

目标环境：

```text
Ubuntu 20.04
ROS2 Foxy
背部 x86 通过网线接入 M20
3D RoboSense 雷达话题 -> /LIDAR/POINTS
Nav2 使用的 2D 激光 -> /scan
Nav2 输出速度 -> /cmd_vel
```

地图文件、真机 TF、里程计、SDK 速度执行链需要按现场情况补齐。

## 0. 模块化接口边界

这个包当前默认走 Nav2 方案，但设计上不要把系统绑死在 Nav2。

默认链路：

```text
RoboSense 3D LiDAR
  -> /LIDAR/POINTS
  -> pointcloud_to_laserscan
  -> /scan
  -> Nav2 AMCL / costmap / planner / controller
  -> /cmd_vel
  -> M20 SDK cmd_vel adapter
  -> M20 底层控制
```

需要重点注意：本包负责到 `/cmd_vel` 为止，**不直接控制机器狗关节或底层运动模式**。实物运动还需要单独的 M20 SDK 速度执行适配层：

```text
/cmd_vel -> m20_cmd_vel_adapter -> M20 SDK
```

这个适配层应负责：

```text
速度限幅
加速度限制
急停
遥控器/人工接管
网络断连保护
Twist 到 M20 SDK 命令格式转换
```

后续如果不用 Nav2，而换成 3D 点云直接避障、voxel map、elevation map、自研 planner 或学习型策略，建议仍然保留统一出口：

```text
其他规划/避障模块 -> /cmd_vel -> m20_cmd_vel_adapter -> M20 SDK
```

更详细的接口契约见：

```text
docs/interface_contract.md
```

## 目录说明

```text
m20_foxy_nav_deploy/
├── config/
│   ├── m20_nav2_foxy_params.yaml      # Foxy 版 Nav2 参数模板
│   └── pointcloud_to_scan.yaml        # 3D 点云转 2D scan 参数参考
├── docs/
│   └── interface_contract.md          # 模块化接口边界
├── launch/
│   ├── lidar_to_scan_test.launch.py   # 只测试 /LIDAR/POINTS -> /scan
│   └── m20_nav_bringup.launch.py      # 雷达转换 + Nav2 bringup
├── maps/
│   └── README.md                      # 地图文件放置说明
└── rviz/
    └── m20_nav2_foxy.rviz             # RViz 观察配置
```

## 1. 上传到随车 x86

把整个包放到随车 x86 的 Foxy 工作空间：

```bash
mkdir -p ~/m20_foxy_ws/src
cp -r m20_foxy_nav_deploy ~/m20_foxy_ws/src/
```

如果是从当前仓库同步，目标结构应类似：

```text
~/m20_foxy_ws/src/m20_foxy_nav_deploy
```

## 2. 安装依赖

在随车 x86 上执行：

```bash
sudo apt update
sudo apt install -y \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-pointcloud-to-laserscan \
  ros-foxy-rviz2 \
  ros-foxy-tf2-ros
```

如果系统缺 DWB 局部规划插件，再补：

```bash
sudo apt install -y \
  ros-foxy-dwb-core \
  ros-foxy-dwb-critics \
  ros-foxy-dwb-plugins
```

## 3. 编译

```bash
cd ~/m20_foxy_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select m20_foxy_nav_deploy
source install/setup.bash
```

建议每个新终端都先执行：

```bash
source /opt/ros/foxy/setup.bash
source ~/m20_foxy_ws/install/setup.bash
```

## 4. 配置 ROS 网络

背部 x86 和 M20 主机必须在同一个 ROS2 DDS 域内。

先确认这些环境变量：

```bash
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
```

通常真机多主机通信应保证：

```bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
```

如果 M20 官方系统使用了其他 `ROS_DOMAIN_ID`，背部 x86 必须保持一致。

## 5. 真机启动前检查

先不要急着让机器狗动。至少确认以下话题和 TF：

```bash
ros2 topic list
ros2 topic echo /LIDAR/POINTS --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

Nav2 完整闭环需要：

```text
/LIDAR/POINTS                sensor_msgs/msg/PointCloud2
/scan                        sensor_msgs/msg/LaserScan，由本包转换得到
/odom                        nav_msgs/msg/Odometry
odom -> base_link             里程计 TF
base_link -> lidar_link       雷达外参 TF
map -> odom                   AMCL 定位后发布
/cmd_vel                     Nav2 或其他规划器输出速度，后续接 M20 SDK adapter
```

如果当前没有 `base_link -> lidar_link`，可以先用静态 TF 临时测试，但上车前必须改成真实外参。

## 6. 只测试 3D 雷达转 2D scan

```bash
ros2 launch m20_foxy_nav_deploy lidar_to_scan_test.launch.py
```

常用参数：

```bash
ros2 launch m20_foxy_nav_deploy lidar_to_scan_test.launch.py \
  cloud_topic:=/LIDAR/POINTS \
  scan_topic:=/scan \
  target_frame:=base_link \
  min_height:=-0.30 \
  max_height:=0.30
```

如果没有雷达 TF，可以临时发布静态 TF：

```bash
ros2 launch m20_foxy_nav_deploy lidar_to_scan_test.launch.py \
  publish_static_lidar_tf:=true \
  lidar_x:=0.0 \
  lidar_y:=0.0 \
  lidar_z:=0.0 \
  lidar_roll:=0.0 \
  lidar_pitch:=0.0 \
  lidar_yaw:=0.0
```

另开终端检查：

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

如果 `/scan` 很稀疏，优先调：

```text
min_height
max_height
range_min
range_max
target_frame
base_link -> lidar_link TF
```

## 7. 放入地图

把你后续补充的二维栅格地图放到：

```text
~/m20_foxy_ws/src/m20_foxy_nav_deploy/maps/
```

推荐命名：

```text
maps/m20_site_map.yaml
maps/m20_site_map.pgm
```

也可以不放在包里，启动时直接传绝对路径。

## 8. 启动 Nav2

确认地图、雷达、里程计、TF 都准备好后：

```bash
ros2 launch m20_foxy_nav_deploy m20_nav_bringup.launch.py \
  map:=/home/你的用户名/m20_foxy_ws/src/m20_foxy_nav_deploy/maps/m20_site_map.yaml
```

如果要指定雷达转换参数：

```bash
ros2 launch m20_foxy_nav_deploy m20_nav_bringup.launch.py \
  map:=/home/你的用户名/m20_foxy_ws/src/m20_foxy_nav_deploy/maps/m20_site_map.yaml \
  use_lidar_to_scan:=true \
  cloud_topic:=/LIDAR/POINTS \
  scan_topic:=/scan \
  target_frame:=base_link \
  min_height:=-0.30 \
  max_height:=0.30
```

如果你的系统已经由官方驱动或其他节点发布了 `/scan`，可以关闭本包的转换：

```bash
ros2 launch m20_foxy_nav_deploy m20_nav_bringup.launch.py \
  map:=/path/to/m20_site_map.yaml \
  use_lidar_to_scan:=false
```

## 9. RViz 操作顺序

RViz 打开后按这个顺序：

1. 看 `TF` 是否有 `map -> odom -> base_link -> lidar_link`。
2. 看 `/map` 是否正常显示。
3. 看 `/scan` 是否贴合地图障碍物。
4. 用 `2D Pose Estimate` 给 AMCL 初始位姿。
5. 看机器人位姿是否稳定贴在地图上。
6. 先不要接实物运动，观察 `/cmd_vel` 是否合理。
7. 确认安全后，再接入 M20 SDK 的速度执行链。
8. 用 `Nav2 Goal` 下发短距离目标。

检查速度输出：

```bash
ros2 topic echo /cmd_vel
```

此时仍然只是确认导航层会输出速度。是否让实物运动，取决于你后续接入的 M20 SDK adapter 是否已经完成限速、急停和人工接管。

## 10. 安全建议

第一次上实物时建议：

```text
1. 先悬空或架起机器狗，只看 /cmd_vel。
2. 再低速空旷场地测试。
3. 保留遥控器/急停/人工接管。
4. 确认 /cmd_vel 的线速度和角速度限幅。
5. 不要直接在复杂环境里首次闭环导航。
```

当前参数已经保守限制：

```text
max_vel_x: 0.30 m/s
max_vel_theta: 0.45 rad/s
robot_radius: 0.32 m
```

后续根据 M20 的实际底层速度接口、机身尺寸、制动距离继续调。

## 11. 常见问题

### 看不到 /scan

检查：

```bash
ros2 topic echo /LIDAR/POINTS --once
ros2 run tf2_ros tf2_echo base_link lidar_link
```

如果 `target_frame:=base_link` 但 TF 不存在，转换节点会无法输出。可以先把 `target_frame:=lidar_link` 或临时发布静态 TF。

### Nav2 不动

重点检查：

```text
/map 是否存在
/scan 是否存在
/odom 是否存在
map -> odom -> base_link 是否连通
AMCL 初始位姿是否设置
/cmd_vel 是否有输出
M20 SDK 是否订阅了正确的 /cmd_vel
```

### 地图能显示但定位飘

通常是：

```text
地图和现场不匹配
雷达外参不准
scan 高度切片不合理
odom 漂移太大
初始位姿给错
```

### PCD 直接生成的栅格图能不能导航

可以做算法验证，但不建议直接当最终真机导航地图。PCD 投影图通常缺少可靠的 free space、墙体厚度和动态障碍清理信息。最终建议使用现场 SLAM/官方建图工具生成 Nav2 标准二维栅格地图，再用这个部署包加载。

### 后续不用 Nav2 怎么办

可以。这个包的 Nav2 部分只是默认实现，不是系统唯一形态。

建议保留两类接口：

```text
感知输入：/LIDAR/POINTS、/odom、TF
执行输出：/cmd_vel
```

中间的 `/scan`、AMCL、costmap、planner、controller 都可以替换。比如：

```text
/LIDAR/POINTS -> 3D obstacle avoidance -> /cmd_vel
/LIDAR/POINTS -> voxel map -> local planner -> /cmd_vel
/LIDAR/POINTS + global map -> hybrid 2D/3D planner -> /cmd_vel
```

只要最后仍然输出 `/cmd_vel`，M20 SDK adapter 和安全层就可以复用。
