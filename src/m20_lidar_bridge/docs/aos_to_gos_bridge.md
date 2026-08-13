# AOS -> GOS 点云桥接验证流程

目标是在 AOS 103 上订阅雷达网 `10.21.33.x` 内的原始 `/LIDAR/POINTS`，经过限频、ROI 和体素降采样后，把轻量障碍云发布到业务网 `10.21.31.x`，供 GOS 104 上的导航算法使用。

## 1. 先验证 AOS 双网卡 DDS

AOS 103 root 终端：

```bash
su
source /opt/robot/scripts/setup_ros2.sh
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/m20_lidar_bridge/install/m20_lidar_bridge/share/m20_lidar_bridge/dds/aos_dual_fastdds.xml

ros2 topic hz /LIDAR/POINTS
ros2 topic pub -r 1 /aos_to_gos_test std_msgs/msg/String "{data: aos-dual-dds-ok}"
```

GOS 104 终端：

```bash
source /opt/robot/scripts/setup_ros2.sh
ros2 topic echo /aos_to_gos_test
```

如果 GOS 收不到 `/aos_to_gos_test`，先不要启动点云桥接。说明 AOS 双网卡 DDS profile 没有打通 31 业务网发布侧。

## 2. 在 AOS 103 编译桥接包

把本包拷到 AOS 的 ROS2 工作空间后：

```bash
cd ~/m20_lidar_bridge
source /opt/ros/foxy/setup.bash
colcon build --packages-select m20_lidar_bridge --symlink-install
```

## 3. 在 AOS 103 启动 ROI 点云桥接

```bash
su
cd ~/m20_lidar_bridge
source /opt/robot/scripts/setup_ros2.sh
source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/m20_lidar_bridge/install/m20_lidar_bridge/share/m20_lidar_bridge/dds/aos_dual_fastdds.xml

ros2 launch m20_lidar_bridge aos_roi_bridge.launch.py
```

默认输入输出：

```text
输入: /LIDAR/POINTS
输出: /m20/perception/obstacle_cloud
```

## 4. 在 GOS 104 验证桥接点云

```bash
source /opt/robot/scripts/setup_ros2.sh
ros2 topic info -v /m20/perception/obstacle_cloud
ros2 topic hz /m20/perception/obstacle_cloud
timeout 5 ros2 topic echo /m20/perception/obstacle_cloud --no-arr
```

期望结果：

```text
频率接近 8 - 10Hz
类型为 sensor_msgs/msg/PointCloud2
width 明显小于原始点云
header.frame_id 与原始 /LIDAR/POINTS 一致，或为配置中的 output_frame
```

## 5. 接入 GOS 导航

GOS 上先不要直接跑完整高速导航，按顺序接：

```text
1. RViz 显示 /m20/perception/obstacle_cloud
2. 录 30 秒 bag，检查延迟、掉帧和点云覆盖范围
3. 点云转 /scan 或接 Nav2 voxel/obstacle layer
4. 低速只读模式验证 costmap
5. 再打开限速后的速度输出
```

M20 真机早期建议速度限制：

```text
线速度 <= 0.2 m/s
角速度 <= 0.3 rad/s
先只做停/减速安全监督，再进入闭环导航
```
