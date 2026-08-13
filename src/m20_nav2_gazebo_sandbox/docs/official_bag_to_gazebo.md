# 官方雷达数据包到 Gazebo 验证场景

## 结论

官方 bag 可以作为开发阶段的地图来源和场景参考，但不能直接当成可交互仿真环境。

原因是 bag 记录的是固定时间段内的传感器数据：

```text
/LIDAR/POINTS
/IMU
```

机器人在 Gazebo 中重新运动时，bag 不会根据新的机器人位姿生成新的雷达观测。因此如果要测试避障，仍需要在 Gazebo 中搭建可碰撞、可被雷达扫描的实体场景。

## 推荐流程

```text
官方 bag
  -> lightning-lm-deep-robotics 建图
  -> 生成 global.pcd / 2D 地图
  -> 参考点云轮廓搭建 Gazebo world
  -> Gazebo 生成 /LIDAR/POINTS
  -> pointcloud_to_laserscan 生成 /scan
  -> SLAM Toolbox / Nav2 costmap
  -> 目标点导航和避障验证
```

## 当前官方数据资产

```text
datasets/m20/lidar_data_bag_0.db3
data/office4f/global.pcd
data/office4f/0.pcd
```

`global.pcd` 是点云地图，适合用于：

- 在 RViz 中检查场景结构。
- 辅助人工搭建 Gazebo 墙体、通道、柱子和障碍物。
- 后续转换为 2D 栅格地图，供 Nav2 定位和全局规划使用。

它不适合直接用于：

- Gazebo 物理碰撞。
- Gazebo 激光雷达实时扫描。
- 闭环测试机器人主动避障。

## 当前验证包的做法

本包先提供一个近似工厂场景：

```text
worlds/official_bag_reference_factory.world
```

该场景不是官方 bag 的精确重建，只是用于优先验证导航系统结构：

```text
/LIDAR/POINTS -> /scan -> costmap -> planner/controller -> /cmd_vel
```

等后续需要更贴近官方点云地图时，再根据 `global.pcd` 逐步调整墙体、通道和障碍物位置。
