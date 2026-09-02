# M20 Navigation Interface Contract

这个文档定义 M20 背部 x86 导航部署包的模块边界。

核心原则：导航方案可以替换，但模块之间只通过 ROS topic、TF、action 和参数连接，不把某一种算法写死。

## 1. 模块分层

```text
传感器层
  -> 感知适配层
  -> 地图/定位层
  -> 规划与避障层
  -> 速度执行适配层
  -> M20 底层控制
```

当前包默认实现：

```text
RoboSense 3D LiDAR
  -> /LIDAR/POINTS
  -> pointcloud_to_laserscan
  -> /scan
  -> Nav2 AMCL / costmap / planner / controller
  -> /cmd_vel
  -> M20 SDK cmd_vel adapter
```

后续可替换实现：

```text
RoboSense 3D LiDAR
  -> /LIDAR/POINTS
  -> 3D obstacle avoidance / voxel map / local planner
  -> /cmd_vel 或自定义控制命令
  -> M20 SDK adapter
```

## 2. 当前包已经包含的模块

### 2.1 感知适配层

文件：

```text
launch/lidar_to_scan_test.launch.py
launch/m20_nav_bringup.launch.py
config/pointcloud_to_scan.yaml
```

输入：

```text
/LIDAR/POINTS  sensor_msgs/msg/PointCloud2
base_link -> lidar_link TF
```

输出：

```text
/scan          sensor_msgs/msg/LaserScan
```

默认算法：

```text
pointcloud_to_laserscan
```

可替换性：

```text
可以替换为 rslidar_laserscan、自己写的点云切片节点、地面分割节点、3D voxel obstacle layer 等。
只要下游仍然给 Nav2 提供 /scan，Nav2 参数可以基本不变。
如果下游不再使用 Nav2，就可以直接绕过 /scan。
```

### 2.2 Nav2 导航层

文件：

```text
launch/m20_nav_bringup.launch.py
config/m20_nav2_foxy_params.yaml
rviz/m20_nav2_foxy.rviz
```

输入：

```text
/map
/scan
/odom
map -> odom
odom -> base_link
base_link -> lidar_link
/goal_pose 或 Nav2 action goal
```

输出：

```text
/cmd_vel      geometry_msgs/msg/Twist
```

默认算法：

```text
AMCL
NavfnPlanner
DWBLocalPlanner
Nav2 local_costmap/global_costmap
```

可替换性：

```text
可以替换 planner_server、controller_server、costmap 插件和定位方式。
也可以关闭 Nav2，使用其他 2D/3D planner，只要最终输出统一速度接口或自定义 SDK 命令。
```

## 3. 这个包没有直接包含的模块

### 3.1 M20 SDK 速度执行适配层

这是实物部署的关键接口，不应和 Nav2 强绑定。

输入建议：

```text
/cmd_vel      geometry_msgs/msg/Twist
```

输出：

```text
M20 官方 SDK 速度控制接口
```

该适配层需要单独实现或接入官方已有节点，职责包括：

```text
1. 订阅 /cmd_vel。
2. 做线速度、角速度限幅。
3. 做加速度限制和低通滤波。
4. 做急停、遥控器接管、网络断连保护。
5. 把 Twist 转成 M20 SDK 实际需要的运动命令。
6. 必要时处理坐标系和运动模式切换。
```

建议输入/输出接口保持：

```text
Nav2 或其他规划器 -> /cmd_vel -> m20_cmd_vel_adapter -> M20 SDK
```

这样后续即使不用 Nav2，新的规划器也只要继续输出 `/cmd_vel`，底层适配层就不用重写。

### 3.2 建图模块

当前包只预留地图目录：

```text
maps/
```

地图来源可以是：

```text
1. 官方建图工具导出的 Nav2 .yaml + .pgm。
2. SLAM Toolbox 建出来的二维栅格地图。
3. PCD 投影生成的工程验证地图。
4. 其他系统生成的 OccupancyGrid。
```

上实物建议优先使用现场建图或官方稳定地图，不建议直接把 PCD 投影图作为最终导航地图。

## 4. 关键接口表

| 接口 | 类型 | 发布者 | 订阅者 | 是否必须 |
| --- | --- | --- | --- | --- |
| `/LIDAR/POINTS` | `sensor_msgs/msg/PointCloud2` | RoboSense 驱动/M20 主机 | 点云转换或 3D 避障模块 | 默认必须 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 点云转换模块 | Nav2 AMCL/costmap | 使用 Nav2 2D 方案时必须 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | map_server/SLAM | AMCL/global_costmap | 地图导航必须 |
| `/odom` | `nav_msgs/msg/Odometry` | M20 底盘/状态估计 | Nav2/controller | 闭环导航必须 |
| `map -> odom` | TF | AMCL/SLAM | Nav2 | 地图定位必须 |
| `odom -> base_link` | TF | M20 底盘/状态估计 | Nav2 | 闭环导航必须 |
| `base_link -> lidar_link` | TF | URDF/robot_state_publisher/static TF | 点云转换/Nav2 | 雷达转换必须 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz/任务层 | Nav2 | 目标点导航需要 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 或其他规划器 | M20 SDK adapter | 实物运动必须 |

## 5. 后续替换成 3D 点云直接避障时怎么改

如果以后不想走 `/LIDAR/POINTS -> /scan -> Nav2`，可以保留下面这些接口：

```text
输入：
/LIDAR/POINTS
/odom
TF
目标点或任务点

输出：
/cmd_vel
```

可以替换掉：

```text
pointcloud_to_laserscan
Nav2 AMCL
Nav2 costmap
Nav2 planner/controller
```

可选方案：

```text
1. 3D voxel map + 3D obstacle avoidance。
2. 自研 local planner 直接消费 PointCloud2。
3. 使用 elevation map / traversability map。
4. 使用全局 2D 地图 + 局部 3D 避障混合方案。
5. 使用学习型策略输出速度，再经过 safety mux。
```

但建议保持最终执行入口：

```text
/cmd_vel -> m20_cmd_vel_adapter -> M20 SDK
```

这样底层安全保护和 SDK 适配可以复用。

## 6. 实物联调建议顺序

```text
1. 只通网络和话题：确认 /LIDAR/POINTS、/odom、TF。
2. 只测感知：确认 /LIDAR/POINTS -> /scan。
3. 只测定位：确认 /map、AMCL、map -> odom。
4. 只测规划：RViz 下发目标，看 /plan 和 costmap。
5. 只看速度：不接狗运动，只 echo /cmd_vel。
6. 接 M20 SDK adapter，但限速、架空、急停。
7. 空旷场地短距离目标。
8. 再进入复杂环境巡航。
```
