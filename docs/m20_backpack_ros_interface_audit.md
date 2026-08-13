# M20 Pro 背部主机 ROS2 接口审计

本文用于整理外接背部主机 `M20piper` 在 `10.21.31.192` 业务网中，当前已经实测或需要继续确认的 ROS2/DDS 接口。这里刻意区分三件事：

1. `topic list` 能看到名字。
2. `topic info -v` 有 publisher/subscriber 端点。
3. `topic hz` 或轻量订阅脚本能收到实质数据。

只有第 3 类才能作为导航、建图、避障或控制闭环输入。

## 1. 当前实测结论

### 1.0 2026-07-21 15:19 背部主机重测快照

本轮重测日志为 `docs/m20_topic_audit_20260721_151914.txt`，实机过程原文追加在 `docs/山猫M20外部主机雷达数据接收配置指南实机过程内容.txt`。本轮环境为：

| 项 | 实测值 |
| --- | --- |
| 主机 | `M20piper` |
| 有线口 | `enp2s0 = 10.21.31.192/24` |
| WiFi | `wlx6c1ff78bc632 = 192.168.112.146/21` |
| ROS | `ROS_DISTRO=foxy`、`ROS_DOMAIN_ID=0`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |

这次审计需要和之前的历史结果分开看：

| 结果 | 结论 |
| --- | --- |
| `/rslidar_points_front` 收到 `PointCloud2`，`frame_id=rslidar_front`，字段 `x,y,z,intensity`，本轮样本约 `15901` 点 | 背部主机前雷达 UDP 组播解码链路已拿到实质点云。 |
| `/rslidar_points_rear` 收到 `PointCloud2`，`frame_id=rslidar_rear`，字段 `x,y,z,intensity`，本轮样本约 `18599` 点 | 背部主机后雷达 UDP 组播解码链路也已拿到实质点云。 |
| `/IMU`、`/ODOM`、`/tf` 被 Python 轻量订阅脚本收到实质消息 | 这些标准 ROS2 话题可作为背部主机侧继续调试入口。 |
| `/ODOM` 样本为 `frame=map`、`child_frame_id` 为空，位置数值很大 | 当前不能直接当成 Nav2 局部里程计使用，需继续确认坐标系语义。 |
| `/LIDAR/POINTS`、`/LIDAR/POINTS2` 未出现在背部主机本轮 topic list | 官方原始点云话题仍未通过业务网直接到背部主机；背部主机当前依赖本地 `rslidar_sdk` 解码的 `/rslidar_points_front/rear`。 |
| `/accumulate_cloud/cloud_base`、`/accumulate_cloud/cloud_gravity`、`/passable_area`、`/impassable_area`、`/traversal_cost` 未出现在本轮 topic list，也未被 Python probe 收到 | 官方处理后感知链路不是本轮默认可用输入；需要重新使能或确认官方节点状态。 |
| `/PASSABLE_AREA_ENABLE` 本轮为 `Publisher count: 1`、`Subscription count: 0` | 本轮它不是可被背部主机发布的使能入口，至少当前没有消费者。 |
| `ros2 topic hz` 检查自定义 `drdds/msg/*` 话题时报 `ModuleNotFoundError: No module named 'drdds'` | 背部主机当前缺少 `drdds` Python 消息模块，`topic info` 能看到端点，但不能 `echo/hz/pub` 这些自定义消息。 |
| `/cmd_vel`、`/m20_inspection/cmd_vel_nav`、`/m20_inspection/cmd_vel_manual`、`/m20_inspection/cmd_vel_safe`、`/m20_inspection/e_stop` 均为 `Unknown topic` | 当前没有启动我们自己的安全速度链路，也没有启动官方 `/cmd_vel` SDK 桥。 |

### 1.0.1 2026-07-21 16:15 工作空间改名与公开 `drdds` 编译快照

背部主机工作空间已经调整为 `~/robodog_nav_system`，当前 `src` 下包含 `drdds`、`rslidar_msg`、`rslidar_sdk`。工作空间改名后，前后雷达本地解码点云重测结果如下：

| 话题 | 实测结果 | 结论 |
| --- | --- | --- |
| `/rslidar_points_front` | 12 秒收到 `120` 帧，约 `9.98Hz`；最后一帧 `frame_id=rslidar_front`，字段 `x,y,z,intensity` | 前雷达 UDP 转播与本地解码链路稳定可用。 |
| `/rslidar_points_rear` | 12 秒收到 `121` 帧，约 `10.07Hz`；最后一帧 `frame_id=rslidar_rear`，字段 `x,y,z,intensity` | 后雷达 UDP 转播与本地解码链路稳定可用。 |

随后将公开版 `drdds` 接口包复制到 `~/robodog_nav_system/src/drdds` 并编译，`colcon build --packages-select drdds --symlink-install` 成功。`ros2 interface show drdds/msg/BatteryData` 和 Python `from drdds.msg import BatteryData, JointsData` 均通过，说明此前的 `ModuleNotFoundError: No module named 'drdds'` 已被解决。

本轮已变成实质可读的 `drdds` 话题：

| 话题 | 实测结果 | 结论 |
| --- | --- | --- |
| `/BATTERY_DATA` | `ros2 topic hz` 约 `1.000Hz`；`ros2 topic echo` 可打印两块电池数据 | 电池状态已可在背部主机直接订阅解析。 |
| `/JOINTS_DATA_10HZ` | `ros2 topic hz` 约 `10Hz` | 低频关节状态已可在背部主机直接订阅解析。 |

编译时出现旧路径 warning：`/home/m20/rslidar_ws/install` 不存在。该 warning 不影响本轮编译结果，但说明背部主机 shell 环境中仍残留旧工作空间路径，建议后续从 `.bashrc` 或自定义环境脚本中清理。

后续曾在背部主机临时新建 `~/m20_ros_env.sh`，用于清理旧 `~/rslidar_ws` 环境残留。该脚本不是官方原有文件，也不是项目必需组件；若系统 `~/.bashrc` 已经正确加载 `/opt/ros/foxy/setup.bash` 并设置 `ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，可以删除该临时脚本和 `.bashrc` 中的 `source ~/m20_ros_env.sh`。实测清理后 `env | grep -E 'rslidar_ws|robodog_nav_system'` 已不再出现旧 `rslidar_ws` 路径。

### 1.0.2 公开版 `drdds` 带来的话题增量

当前复制到背部主机的是官方公开 SDK 中的 `drdds` 接口包，包含 `BatteryData`、`JointsData`、`JointsDataCmd`、`ImuData`、`GamepadData`、`StdMsgInt32`、`StdMsgFloat32`、`StdStatus` 等基础类型。它不是机器狗内部完整 `drdds` 包，因此只能解决一部分自定义消息的解析问题。

已经由本轮实测确认“能实质查看”的增量：

| 话题 | 类型 | 当前状态 | 用途 |
| --- | --- | --- | --- |
| `/BATTERY_DATA` | `drdds/msg/BatteryData` | 已 `hz` 约 `1Hz`，已 `echo` 出两块电池数据 | 电池电压、电流、电量、温度、循环次数等状态监控 |
| `/JOINTS_DATA_10HZ` | `drdds/msg/JointsData` | 已 `hz` 约 `10Hz` | 低频关节状态监控、运动状态记录 |

公开版 `drdds` 已经具备类型定义、下一步可以继续实测的候选：

| 话题 | 类型 | 15:19 端点状态 | 当前建议 |
| --- | --- | --- | --- |
| `/JOINTS_DATA` | `drdds/msg/JointsData` | `Publisher count: 1` | 可继续 `hz/echo --no-arr`，用于确认高频关节状态是否适合记录或监督 |
| `/JOINTS_CMD` | `drdds/msg/JointsDataCmd` | `Publisher count: 1`、`Subscription count: 1` | 只建议监听诊断，不建议背部主机直接发布 |
| `/HES_STATUS` | `drdds/msg/StdMsgInt32` | `Publisher count: 1`、`Subscription count: 1` | 可继续 `hz/echo`，确认状态值语义 |
| `/NAV_STATUS` | `drdds/msg/StdMsgInt32` | `Publisher count: 1` | 可继续 `hz/echo`，用于观察官方导航状态 |
| `/GPS_CFGSYS` | `drdds/msg/StdMsgInt32` | `Publisher count: 1` | 可继续 `hz/echo`，只做状态观察 |
| `/UWB_ENABLE` | `drdds/msg/StdMsgInt32` | `Publisher count: 1` | 可继续 `hz/echo`，不要随意发布开关命令 |

公开版 `drdds` 类型已具备，但当前审计中没有 publisher，暂时不能期望 `echo/hz` 有输出；若后续官方节点启动发布，则可以解析：

| 话题 | 类型 | 15:19 端点状态 |
| --- | --- | --- |
| `/BATTERY_CHARGE_ENABLE` | `drdds/msg/StdMsgInt32` | `Publisher count: 0`、`Subscription count: 1` |
| `/GPS_SYS_MODE` | `drdds/msg/StdMsgInt32` | `Publisher count: 0`、`Subscription count: 1` |
| `/UWB_ODOM_LOST` | `drdds/msg/StdMsgInt32` | `Publisher count: 0`、`Subscription count: 1` |
| `/UWB_ONLINE` | `drdds/msg/StdMsgInt32` | `Publisher count: 0`、`Subscription count: 1` |
| `/CHARGE_STATUS` | `drdds/msg/StdStatus` | `Publisher count: 0`、`Subscription count: 1` |
| `/HEIGHT_MAP_STATUS` | `drdds/msg/StdStatus` | `Publisher count: 0`、`Subscription count: 1` |
| `/OOA_STATUS` | `drdds/msg/StdStatus` | `Publisher count: 0`、`Subscription count: 1` |
| `/TERRAIN_CLASSIFIER_STATUS` | `drdds/msg/StdStatus` | `Publisher count: 0`、`Subscription count: 1` |

仍然不能靠公开版 `drdds` 解析的话题类型包括：`CpuData`、`Exception`、`FaultStatus`、`Gait`、`PlannerStatus`、`NavSat`、`GridsID`、`Steer`、`LedStatusLight`、`AiryLidarStatus`、`LocationStatus`、`MotionInfo`、`MotionState`、`MotionStatus`、`NavCmd`。这些需要后续从 AOS/NOS/GOS 中找到机器狗内部完整 `.msg` 定义，再在背部主机重新编译同名 `drdds`。

### 1.0.3 2026-07-21 16:53 手工话题审计快照

本轮按手工命令重测，不再依赖临时脚本。环境已清理干净，`env | grep -E 'rslidar_ws|robodog_nav_system'` 只剩 `~/robodog_nav_system`，没有旧 `~/rslidar_ws` 路径。`ROS_DISTRO=foxy`、`ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`。

本轮已经确认有实质数据的输入：

| 话题 | 类型 | 频率/样本 | 当前用途判断 |
| --- | --- | --- | --- |
| `/IMU` | `sensor_msgs/msg/Imu` | 约 `200Hz`，`frame_id` 为空 | 主 IMU，可用于状态估计、姿态监督、感知时间同步检查。 |
| `/ODOM` | `nav_msgs/msg/Odometry` | 约 `10Hz`，`frame=map`、`child_frame_id` 为空，位置数值很大 | 可读，但坐标语义异常，暂不建议直接作为 Nav2 局部里程计。 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 约 `10Hz`，实测边为 `map->base_link` | 可用于 RViz 显示和坐标关系检查。 |
| `/LIDAR/IMU201` | `sensor_msgs/msg/Imu` | 约 `200Hz`，`frame=lidar_link` | 前雷达 IMU/诊断输入。 |
| `/LIDAR/IMU202` | `sensor_msgs/msg/Imu` | 约 `200Hz`，`frame=lidar_link` | 后雷达 IMU/诊断输入。 |
| `/rslidar_points_front` | `sensor_msgs/msg/PointCloud2` | 约 `10Hz`，`frame=rslidar_front`，样本约 `1x15626/1x15702` 点，字段 `x,y,z,intensity` | 前雷达本地解码点云，当前背部主机三维感知主输入。 |
| `/rslidar_points_rear` | `sensor_msgs/msg/PointCloud2` | 约 `10Hz`，`frame=rslidar_rear`，样本约 `1x21096/1x21472` 点，字段 `x,y,z,intensity` | 后雷达本地解码点云，当前背部主机三维感知主输入。 |
| `/BATTERY_DATA` | `drdds/msg/BatteryData` | 约 `1Hz`，`echo --no-arr` 可见两块电池序列 | 电池监控可用；若要看明细不要加 `--no-arr`。 |
| `/JOINTS_DATA` | `drdds/msg/JointsData` | 约 `190Hz`，`echo --no-arr` 可见 16 关节数组 | 高频关节状态可用，但数据量很大，建议程序订阅或短时抽样，不建议长时间直接 `echo`。 |
| `/JOINTS_DATA_10HZ` | `drdds/msg/JointsData` | 约 `8.5-10Hz`，`echo --no-arr` 可见 16 关节数组 | 低频关节状态可用，更适合日志和健康监控。 |
| `/LOCATION_STATUS/MATCHING_ERROR` | `std_msgs/msg/Float64MultiArray` | 约 `10Hz` | 定位匹配误差类状态可读，可用于定位健康监督。 |

本轮确认仍不可作为背部主机输入：

| 话题 | 结果 | 结论 |
| --- | --- | --- |
| `/LIDAR/POINTS`、`/LIDAR/POINTS2` | `topic hz` 提示未发布，Python probe 12 秒未收到 | 背部主机当前仍不能直接依赖官方原始点云话题。 |
| `/LOC_BODY_POINTS` | `topic hz` 提示未发布，Python probe 12 秒未收到 | 当前不是可用点云输入。 |
| `/LIO_ODOM`、`/LIO_ODOM_HIGH_FREQUENCY` | `topic hz` 提示未发布，Python probe 12 秒未收到 | 当前不可作为定位输入。 |
| `/traversal_cost`、`/GRID_MAP` | topic list 可见，但 Python probe 12 秒未收到 | 当前不能只凭 topic list 视作可用栅格输入。 |

本轮确认“可见但缺内部消息定义，暂不能在背部主机解析”的话题：

| 话题 | 类型 | 结果 |
| --- | --- | --- |
| `/LIDAR/STATUS` | `drdds/msg/AiryLidarStatus` | `ros2 topic hz` 报 `drdds.msg` 无 `AiryLidarStatus`，需要完整内部 `drdds`。 |
| `/MOTION_INFO`、`/MOTION_STATE`、`/MOTION_STATUS`、`/FAULT_STATUS`、`/LOCATION_STATUS`、`/PLANNER_STATUS`、`/GLOBAL_PLANNER_STATUS`、`/CPU_104`、`/CPU_106` 等 | `drdds/msg/*` 内部扩展类型 | topic list 可见，但公开版 `drdds` 不含这些类型，暂不能 `hz/echo`。 |

本轮手工 `echo` 发现 `/HES_STATUS`、`/NAV_STATUS`、`/GPS_CFGSYS`、`/UWB_ENABLE`、`/BATTERY_CHARGE_ENABLE`、`/CHARGE_STATUS` 等仅打印出类型，未在 5 秒窗口内收到实质消息。它们不能因为类型可解析就直接视作可用输入。

### 1.1 背部主机可订阅的实质数据

| 类别 | 话题 | 类型 | 当前实测状态 | 适合用途 |
| --- | --- | --- | --- | --- |
| 主 IMU | `/IMU` | `sensor_msgs/msg/Imu` | AOS/GOS/背部链路中均可见；实测约 `200Hz`，时间戳稳定；官方对外接口推荐使用该话题 | 姿态、定位、点云处理、状态监控 |
| 里程计 | `/ODOM` | `nav_msgs/msg/Odometry` | 背部主机/GOS 可见，实测约 `10Hz`；但 `frame=map`、`child_frame_id` 为空、位置数值很大 | 低速任务/监督候选，需先弄清坐标语义 |
| TF | `/tf` | `tf2_msgs/msg/TFMessage` | 背部主机可见，实测约 `10Hz`；当前 probe 看到 `map->base_link` | 坐标关系检查、RViz 显示 |
| 前雷达本地解码点云 | `/rslidar_points_front` | `sensor_msgs/msg/PointCloud2` | 2026-07-21 16:53 实测约 `10Hz`；`frame_id=rslidar_front`，字段 `x,y,z,intensity` | 前向三维感知、建图/避障输入候选 |
| 后雷达本地解码点云 | `/rslidar_points_rear` | `sensor_msgs/msg/PointCloud2` | 2026-07-21 16:53 实测约 `10Hz`；`frame_id=rslidar_rear`，字段 `x,y,z,intensity` | 后向三维感知、近场安全监督输入候选 |
| 电池 | `/BATTERY_DATA` | `drdds/msg/BatteryData` | 2026-07-21 16:15 已实测约 `1Hz` 且可 echo 两块电池数据 | 电量、健康状态监控 |
| 关节状态 | `/JOINTS_DATA` | `drdds/msg/JointsData` | 2026-07-21 16:53 已实测约 `190Hz` 且可 echo | 高频运动状态监控、SDK 状态输入 |
| 低频关节状态 | `/JOINTS_DATA_10HZ` | `drdds/msg/JointsData` | 2026-07-21 16:53 已实测约 `8.5-10Hz` 且可 echo | 低频监控 |
| 雷达 IMU | `/LIDAR/IMU201`、`/LIDAR/IMU202` | `sensor_msgs/msg/Imu` | 2026-07-21 16:53 实测约 `200Hz`，`frame=lidar_link` | 雷达状态、诊断 |
| 定位匹配误差 | `/LOCATION_STATUS/MATCHING_ERROR` | `std_msgs/msg/Float64MultiArray` | 2026-07-21 16:53 实测约 `10Hz` | 定位健康监督 |
| 官方处理后点云 | `/accumulate_cloud/cloud_base` | `sensor_msgs/msg/PointCloud2` | 历史轮次曾输出；2026-07-21 15:19 背部重测未出现在 topic list | 低速局部感知候选，需重新使能确认 |
| 官方处理后点云 | `/accumulate_cloud/cloud_gravity` | `sensor_msgs/msg/PointCloud2` | 历史轮次曾输出；2026-07-21 15:19 背部重测未出现在 topic list | 三维局部感知验证候选，需重新使能确认 |
| 可通行点云 | `/passable_area` | `sensor_msgs/msg/PointCloud2` | 历史轮次曾输出；2026-07-21 15:19 背部重测未出现在 topic list | 可通行区域可视化/监督，需重新使能确认 |
| 不可通行点云 | `/impassable_area` | `sensor_msgs/msg/PointCloud2` | 历史轮次曾输出；2026-07-21 15:19 背部重测未出现在 topic list | 低速障碍监督候选，需重新使能确认 |
| 局部代价图 | `/traversal_cost` | `nav_msgs/msg/OccupancyGrid` | 历史轮次曾输出；2026-07-21 15:19 背部重测未出现在 topic list | 低速局部避障/监督层候选，需重新使能确认 |
| 官方状态 | `/MOTION_STATE`、`/MOTION_STATUS`、`/NAV_STATUS`、`/PLANNER_STATUS`、`/FAULT_STATUS`、`/LOCATION_STATUS` | 待逐个确认 | topic list 可见 | 状态机、UI、安全监督 |
| 资源状态 | `/CPU_103`、`/CPU_104`、`/CPU_106` | 待逐个确认 | topic list 可见 | 三主机负载监控 |

补充：官方文档说明系统内部还存在 `/IMU_YESENSE`、`/IMU_DATA`、`/IMU_DATA_10HZ` 等 IMU 相关话题，但这些不在对外接口清单中，不建议二次开发依赖。2026-07-21 背部主机实测 `/IMU_DATA` 不可见，`ros2 topic type /IMU_DATA` 无输出；这与官方“对外使用 `/IMU`”的说明一致。

结论：背部主机已经具备“标准状态话题 + 前后雷达本地解码点云 + RViz/日志”的开发价值。官方处理后感知链路在本轮不是默认可用，需要单独使能和复测。

### 1.2 背部主机当前不应直接依赖的输入

| 话题 | 当前判断 | 原因 |
| --- | --- | --- |
| `/LIDAR/POINTS` | 不建议作为背部主机主输入 | AOS root 下可达约 `9-10Hz`；业务网/GOS/背部主机默认未稳定获得原始点云。后续本地解码 UDP 组播可另行发布 `/rslidar_points_front`、`/rslidar_points_rear`。 |
| `/LIDAR/POINTS2` | 未确认可用 | topic 可能可见但实测常无输出或未完成语义确认。 |
| `/LOC_BODY_POINTS` | 未确认可用 | 曾出现 publisher count 为 0 或 hz 无输出；但使能后下游处理链可输出，说明不能仅凭该话题判断链路完全失效。 |
| `/LIO_ODOM`、`/LIO_ODOM_HIGH_FREQUENCY` | 未确认实时可用 | 话题名可见，但本轮短时间 hz 无输出。需确认官方定位/建图状态。 |

### 1.3 背部主机可发布的非运动控制话题

| 话题 | 类型 | 作用 | 风险 |
| --- | --- | --- | --- |
| `/PASSABLE_AREA_ENABLE` | `std_msgs/msg/Int32` | 历史轮次曾被当作使能入口测试；但 2026-07-21 15:19 背部重测显示 `Publisher count: 1`、`Subscription count: 0` | 当前不能再当作可靠使能入口。需先找到真正 subscriber 或官方节点状态。 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 常见定位初始化入口，具体是否被官方节点消费需确认 subscriber | 中等。错误初值会影响定位 |
| `/target_goal` | 需确认 | 官方导航/规划目标候选话题，需先确认类型和订阅者 | 中等。可能触发官方规划 |
| `/planner_mode` | 需确认 | 官方规划模式候选话题，需确认类型和语义 | 中等。可能改变官方导航状态 |
| `/UWB_ENABLE` | 需确认 | UWB 开关候选话题，需确认类型和语义 | 中等。不要随意发布 |

### 1.4 背部主机可发布的运动控制接口

推荐分层如下：

```text
上层导航/任务:
  发布 /m20_inspection/cmd_vel_nav

人工/遥控:
  发布 /m20_inspection/cmd_vel_manual

急停:
  发布 /m20_inspection/e_stop

项目安全层:
  cmd_vel_safety_mux
    /m20_inspection/cmd_vel_nav
    /m20_inspection/cmd_vel_manual
    /m20_inspection/e_stop
      -> /m20_inspection/cmd_vel_safe

运动适配层:
  motion_adapter_node
    /m20_inspection/cmd_vel_safe
      -> /cmd_vel

官方 SDK 导航桥:
  m20_sdk_deploy rl_deploy_cmdvel
    /cmd_vel
      -> 官方状态机 + RL policy
      -> /JOINTS_CMD
```

推荐背部主机后续发布：

| 话题 | 类型 | 推荐发布者 | 说明 |
| --- | --- | --- | --- |
| `/m20_inspection/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2、MPPI、SCAN-Planner 输出桥、任务状态机 | 最推荐。经过安全 mux、限速、加速度限制和超时归零 |
| `/m20_inspection/cmd_vel_manual` | `geometry_msgs/msg/Twist` | 键盘/遥控测试节点 | 手动优先级默认高于导航 |
| `/m20_inspection/e_stop` | `std_msgs/msg/Bool` | 安全节点/UI/遥控器 | `true` 急停，`false` 解除 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 仅限联调或确认安全层未运行时 | 会直接进入 `rl_deploy_cmdvel`，绕过项目安全 mux，不建议作为常规入口 |

注意：2026-07-21 15:19 重测时，上述 `/cmd_vel` 与 `/m20_inspection/*` 话题均为 `Unknown topic`，说明当时没有启动本项目安全控制链路，也没有启动官方 `m20_cmd_vel_interface`。要做运动烟测，必须先启动这些节点，再看 topic info 是否出现对应 publisher/subscriber。

不建议背部主机直接发布：

| 话题 | 类型 | 原因 |
| --- | --- | --- |
| `/JOINTS_CMD` | `drdds/msg/JointsDataCmd` | 这是 16 关节底层命令，应该由官方 SDK/RL 策略发布。导航算法直接发它风险极高 |
| `/NAV_CMD`、`/CHARGE_CMD`、`/GAIT`、`/STEER` | 待确认 | 属于官方控制/模式接口，语义未完全确认前不要发布 |

## 2. 背部主机实测审计指令

先确保环境一致：

```bash
cd ~/robodog_nav_system
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

### 2.1 一次性列出所有话题、类型、端点数量

```bash
for t in $(ros2 topic list | sort); do
  ty=$(ros2 topic type "$t" 2>/dev/null)
  echo "===== $t ====="
  echo "type: $ty"
  ros2 topic info "$t" 2>/dev/null
done
```

### 2.2 验证关键输入是否有实质数据

```bash
for t in \
  /IMU /IMU_YESENSE /ODOM /tf \
  /BATTERY_DATA /JOINTS_DATA /JOINTS_DATA_10HZ \
  /LIDAR/IMU201 /LIDAR/IMU202 /LIDAR/STATUS \
  /LIDAR/POINTS /LIDAR/POINTS2 /LOC_BODY_POINTS \
  /LIO_ODOM /LIO_ODOM_HIGH_FREQUENCY
do
  echo "===== $t ====="
  ros2 topic type "$t"
  timeout 8 ros2 topic hz "$t"
done
```

### 2.3 使能并验证官方处理后感知链路

```bash
ros2 topic pub --once /PASSABLE_AREA_ENABLE std_msgs/msg/Int32 "{data: 1}"

for t in \
  /accumulate_cloud/cloud_base \
  /accumulate_cloud/cloud_gravity \
  /passable_area \
  /impassable_area \
  /traversal_cost \
  /accumulate_cloud/status_code \
  /passable_status_code
do
  echo "===== $t ====="
  ros2 topic type "$t"
  timeout 8 ros2 topic hz "$t"
done
```

### 2.4 打印点云/代价图元数据

```bash
python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

topics_pc = [
    '/accumulate_cloud/cloud_base',
    '/accumulate_cloud/cloud_gravity',
    '/passable_area',
    '/impassable_area',
]

class Probe(Node):
    def __init__(self):
        super().__init__('m20_backpack_meta_probe')
        for topic in topics_pc:
            self.create_subscription(PointCloud2, topic, lambda msg, t=topic: self.pc_cb(t, msg), 10)
        self.create_subscription(OccupancyGrid, '/traversal_cost', self.grid_cb, 10)

    def pc_cb(self, topic, msg):
        fields = ','.join(f.name for f in msg.fields)
        print(f'[{topic}] frame={msg.header.frame_id} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} width={msg.width} height={msg.height} point_step={msg.point_step} fields={fields}', flush=True)

    def grid_cb(self, msg):
        info = msg.info
        print(f'[/traversal_cost] frame={msg.header.frame_id} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} size={info.width}x{info.height} res={info.resolution} origin=({info.origin.position.x},{info.origin.position.y},{info.origin.position.z}) data_len={len(msg.data)}', flush=True)

rclpy.init()
node = Probe()
end = time.time() + 8.0
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
PY
```

### 2.5 验证项目安全控制入口

只在架空、轮子离地、或明确处于安全测试状态时做：

```bash
# 观察链路
ros2 topic info /m20_inspection/cmd_vel_nav
ros2 topic info /m20_inspection/cmd_vel_safe
ros2 topic info /cmd_vel

# 发布急停
ros2 topic pub --once /m20_inspection/e_stop std_msgs/msg/Bool "{data: true}"

# 解除急停
ros2 topic pub --once /m20_inspection/e_stop std_msgs/msg/Bool "{data: false}"

# 极小速度烟测，必须有人手动看护
ros2 topic pub --rate 5 /m20_inspection/cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停止时按 `Ctrl-C` 后，安全 mux 和 motion adapter 都应在超时后发布零速度。也可以主动发布：

```bash
ros2 topic pub --once /m20_inspection/cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 3. 当前开发建议

1. 背部主机短期定位：监控、RViz、bag、UI、任务状态机、低速安全监督。
2. 背部主机可先用 `/traversal_cost` 或 `/impassable_area` 做保守减速/停车监督，不要直接承担高速动态避障主闭环。
3. 导航速度输出优先走 `/m20_inspection/cmd_vel_nav`，不要直接发 `/JOINTS_CMD`。
4. 真正要做三维高频避障，仍需要解决原始 `/LIDAR/POINTS` 或等价高频障碍云到 GOS/背部主机的问题。
5. 若要直接控制实机运动，必须确认 SDK 模式、官方 `rl_deploy_cmdvel`、项目安全 mux 和急停链路都已经启动并可观测。
