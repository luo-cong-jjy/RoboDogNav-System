# M20 Nav2 System

本包提供 M20 Pro 的二维导航仿真与后续实机接入基础。导航层统一使用 Nav2、RViz、二维栅格地图、代价地图和动态避障；底层可选择 Gazebo、MuJoCo 或 AOS 实机后端。

## 仿真与实机边界

导航层保持一套实现，后端按启动入口切换，不移动现有目录或复制第二套导航代码：

```text
RViz / Nav2 / 地图 / 代价地图 / 动态避障
                    |
       /scan  /odom  /tf  /cmd_vel
                    |
       +------------+-------------+
       |                          |
   Gazebo / MuJoCo             AOS 实机
   模拟传感器与运动层          真实雷达、LIO、官方 SDK
```

| 场景 | 启动入口 | 后端 |
| --- | --- | --- |
| Gazebo 仿真 | `factory_navigation.launch.py` | Gazebo 机器人、已保存二维地图、AMCL、Nav2 和 RViz |
| Gazebo 实时建图导航 | `factory_slam_navigation.launch.py` | Gazebo 2D 雷达、slam_toolbox 和 Nav2，RViz目标点驱动 |
| Gazebo 已保存地图导航 | `factory_saved_map_navigation.launch.py` | Gazebo 传感器 + 保存的二维地图 + AMCL + Nav2，RViz目标点驱动 |
| MuJoCo 仿真 | `m20_mujoco_navigation.launch.py` | 官方 M20 运动层与模拟传感器 |
| MuJoCo 仅查看 | `m20_mujoco_viewer_only.launch.py` | 运动模型和工厂场景显示 |
| AOS 实机 | `m20_hardware_navigation.launch.py` | 真实雷达、LIO、官方 SDK |

实机入口只替换后端，不替换 Nav2 的 `/cmd_vel` 接口。任何时刻只能有一个节点发布 `/JOINTS_CMD`；实机启动时必须关闭 MuJoCo、模拟雷达和模拟 odom。

## AOS 实机部署步骤

### 背部主机部署边界

不要把整个工作空间的 `src`、`build` 或 `install` 目录复制到 AOS。当前源码按
ROS 包边界部署：必须包为 `m20_nav2_system`、`m20_nav2_description` 和
`m20_nav2_locomotion`；若采用 MuJoCo 已验证的 RL 后端，再额外部署官方
`M20_sdk_deploy` 及其 `drdds` 依赖。`m20_nav2_backend` 仅用于 MuJoCo，不部署
到实机。Gazebo world、Gazebo 插件、MuJoCo viewer 和模拟雷达也不属于实机运行集。

可用 `hardware.repos` 作为依赖清单模板导入厂商消息包；其中仓库地址和版本必须
替换为 AOS 实际批准的版本，不能盲目使用 `main` 分支。AOS 上建议关闭 Gazebo
插件构建：

```bash
colcon build --symlink-install --cmake-args -DBUILD_GAZEBO_PLUGINS=OFF
```

这样导航包仍保留同一套 launch、地图、Nav2 配置和接口，但不会在背部主机编译
四轮 Gazebo 插件。仿真开发机保持默认 `BUILD_GAZEBO_PLUGINS=ON`。

### 实机阶段 1：真实环境建图

先启动 AOS 官方传感器和里程计节点，确认它们发布 `/scan`、`/odom` 以及
`odom -> base_link` TF。随后只启动本包的建图入口：

```bash
ros2 launch m20_nav2_system m20_hardware_mapping.launch.py
```

该入口不启动 Gazebo、MuJoCo、AMCL 或 Nav2；它启动 `slam_toolbox`、RViz，
以及可选的官方 SDK 速度适配器。AOS 的真实雷达、里程计和 `odom -> base_link`
TF 由官方传感器/LIO 节点提供。默认 `start_sdk=true`，因此可以通过 `/cmd_vel`
进行人工运动；若 SDK 已在其他终端启动，应将 `start_sdk:=false`，避免重复连接。
用 RViz 的 `2D Pose Estimate` 设置初始位姿，驱动机器狗覆盖环境。
建图完成后，在另一个终端保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f /path/to/m20_factory_real
```

保存得到的 `.yaml` 和 `.pgm` 是阶段 2 的输入。保存前必须确认地图覆盖完整、
闭环正常且 `map -> odom` 稳定。

实机部署按阶段进行，每一阶段通过后才能进入下一阶段。AOS 上的 ROS 发行版、SDK 授权和网络配置以厂家实际环境为准；开发机通常使用 Humble，AOS 可能使用 Foxy，不能直接混用工作空间的 `install/`。

### 1. 只验证官方 SDK

在 AOS 上单独编译并启动官方 SDK（首次使用前按厂家要求完成固件升级和 SDK 授权）：

```bash
source /opt/ros/<aos-ros-distro>/setup.bash
cd ~/sdk_deploy
colcon build --packages-select m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm
source install/setup.bash
ros2 run m20_sdk_deploy rl_deploy
```

确认官方状态话题稳定：

```text
/IMU 或厂家实际 IMU 话题
/JOINTS_DATA
/BATTERY_DATA
```

此阶段不启动 Nav2，不发布 `/cmd_vel`，不连接自动导航。确认手柄/官方状态机能够安全站立、趴下和退出 SDK 模式后再继续。

官方键盘模式的逐项验收动作如下。每次只短按一次，并在空旷区域进行：

| 按键 | 预期动作 |
| --- | --- |
| `z` | 站立/默认姿态 |
| `c` | 进入强化学习控制模式 |
| `x` | 趴下 |
| `w` | 前进 |
| `s` | 后退 |
| `a` | 左移 |
| `d` | 右移 |
| `q` | 顺时针原地旋转 |
| `e` | 逆时针原地旋转 |

每次动作都要确认方向正确，松开按键后能够停止；`z -> c -> w/q/e -> x` 应能完整执行。手柄模式下对应为：`L1` 站立、`L2` 进入 RL 控制、`R1` 趴下、`R2` 关节阻尼，左摇杆负责前后，右摇杆负责旋转。当前 Gazebo 差速模型不支持横向平移。必须保留物理急停。

确认官方反馈持续发布：

```bash
ros2 topic hz /JOINTS_DATA
ros2 topic hz /IMU_DATA
ros2 topic echo /BATTERY_DATA --once
ros2 topic info /JOINTS_CMD -v
```

### 2. 验证导航速度适配器

使用本地 SDK 扩展编译 `rl_deploy_cmdvel`，并确认它是唯一的 `/JOINTS_CMD` 发布者：

```bash
ros2 run m20_sdk_deploy rl_deploy_cmdvel
ros2 topic info /JOINTS_CMD -v
```

先在空旷区域以人工急停为保障，向 `/cmd_vel` 输入极低速的前进、旋转和停止指令。若出现多个控制源、状态机未授权、速度方向异常或无法立即停止，必须停止部署。

逐项发送以下测试指令。当前 Gazebo 差速模型使用 `linear.x` 前后和 `angular.z` 偏航，`linear.y` 不参与运动；先使用低速值，确认方向后再逐步提高。

```bash
# 前进、后退
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左移、右移
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.05, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: -0.05, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 逆时针、顺时针原地旋转
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.15}}"

# 停止
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

还要验证命令超时保护：用 `ros2 topic pub -r 10` 连续发送低速指令，按 `Ctrl+C` 停止发布，确认机器人自动回零；不能依赖手动发送零速度才能停下。

### 3. 接入真实雷达、里程计和二维地图定位

Nav2 不默认使用 LIO。使用已有二维栅格地图导航时，本包实机入口启动 Nav2 的 `map_server + AMCL`：AMCL 使用 `/scan`、二维地图和里程计完成全局定位，并发布 `map -> odom`。底盘侧仍必须提供连续的 `/odom` 和 `odom -> base_link`；它们可以来自 AOS 自带里程计、轮速/IMU 融合、LIO 或其他状态估计器，并不限定算法。

如果是边建图边导航，可改用 SLAM Toolbox 等二维 SLAM，由 SLAM 节点建立地图并提供全局定位，此时不再同时启动 AMCL。LIO 只有在选它作为里程计/定位来源时才需要，不是 Nav2 的默认定位算法。

关闭所有仿真传感器后启动真实雷达驱动和选定的里程计来源，先只检查数据，不放行运动：

```text
/scan
/odom
AMCL: map -> odom
底盘里程计: odom -> base_link
机器人描述: base_link -> lidar_link
```

必须确认坐标系方向、时间戳、雷达 QoS 和初始位姿正确。启动实机入口后，需要在 RViz 使用 `2D Pose Estimate` 给 AMCL 初始位姿，直到粒子云收敛且机器人移动时在地图中的姿态稳定。实机地图应使用实测/验证过的地图，不能使用 Gazebo 或 MuJoCo 的模拟地图作为最终地图。

### 4. 接入 Nav2

启动实机 Nav2 配置时，只保留真实输入和统一速度接口：

```text
真实雷达 -> /scan
真实里程计 -> /odom 与 odom -> base_link TF
AMCL       -> map -> odom TF
Nav2      -> /cmd_vel
/cmd_vel  -> 安全过滤 -> rl_deploy_cmdvel
```

启动顺序应为：真实传感器与定位、地图服务、Nav2、RViz、速度安全层，最后由人工确认后解除运动锁。初次测试限制线速度和角速度，并安排物理急停人员。

### 5. 停止与回退

退出时先取消导航目标并发布零速度，再按官方流程退出 SDK 模式；不要直接杀掉唯一的运动节点。发现 TF 丢失、雷达停止、定位跳变、命令超时或 `/JOINTS_CMD` 出现竞争时，立即进入软急停并回到第 1 步重新验证。

## 启动入口

实机专用入口（不启动 Gazebo、MuJoCo 或模拟传感器；`start_sdk` 默认关闭）：

```bash
ros2 launch m20_nav2_system m20_hardware_navigation.launch.py \
  map:=/path/to/verified_real_map.yaml
```

确认真实 `/scan`、`/odom` 和 TF 稳定、急停有效后，再显式加入
`start_sdk:=true` 放行官方 `rl_deploy_cmdvel`。真实设备话题可用
`scan_topic:=...` 和 `odom_topic:=...` 覆盖。

```bash
# Gazebo：默认使用已保存地图 + AMCL，验证完整导航链路
ros2 launch m20_nav2_system factory_navigation.launch.py

# 建图完成后，将地图直接保存为本包资产
mkdir -p src/m20_nav2_system/navigation/m20_nav2_system/maps/factory
ros2 run nav2_map_server map_saver_cli \
  -f src/m20_nav2_system/navigation/m20_nav2_system/maps/factory/m20_factory_slam

# Gazebo + 官方 slam_toolbox 实时建图 + Nav2（不读取预置地图）
ros2 launch m20_nav2_system factory_slam_navigation.launch.py
# 上述入口默认同时启动键盘控制；如需关闭键盘节点：
# ros2 launch m20_nav2_system factory_slam_navigation.launch.py use_keyboard:=false

# Gazebo + 已保存地图 + AMCL + Nav2（验证建图后的完整导航）
ros2 launch m20_nav2_system factory_saved_map_navigation.launch.py

# MuJoCo：官方 M20 运动层 + 模拟传感器 + Nav2 + RViz
ros2 launch m20_nav2_system m20_mujoco_navigation.launch.py

# 仅查看 MuJoCo 运动模型和工厂场景
ros2 launch m20_nav2_system m20_mujoco_viewer_only.launch.py

# Nav2 已运行后启动巡检任务
ros2 launch m20_nav2_system factory_inspection_mission.launch.py
```

Gazebo 和 MuJoCo 是互斥后端，不能同时启动。MuJoCo Viewer 只用于运动层观察，不参与 Nav2 规划；动态障碍物的权威轨迹由场景模拟器发布，Viewer 仅镜像显示。

## 统一接口

```text
传感器输入：/scan
定位输入：  /odom
坐标变换：  map -> odom -> base_link -> base_scan
导航输出：  /cmd_vel
```

仿真时 `/scan` 和 `/odom` 由 MuJoCo/栅格模拟桥提供；实机时替换为真实雷达和 LIO，不修改 Nav2。实机速度必须经过安全过滤后再进入 `rl_deploy_cmdvel`，最终由官方 SDK 发布 `/JOINTS_CMD`。任何时刻只能存在一个 `/JOINTS_CMD` 发布者。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/robodog_nav_system
colcon build --packages-select m20_nav2_system --symlink-install
source install/setup.bash
```

## MuJoCo 工厂场景

`worlds/factory_environment.world` 是原 MuJoCo 验证场景，也是 Gazebo 保存地图导航
共用的唯一动态工厂场景定义。启动 MuJoCo 时会自动转换并加载：

- 静态围墙和工作台：参与 MuJoCo 碰撞并在 Viewer 中显示。
- 动态障碍物：以可移动 mocap 圆柱显示，位置与 RViz/雷达模拟器同步；数量、尺寸、轨迹
  和时间尺度与 Gazebo 一致。
- 官方 M20 地面：保留 MJCF 自带棋盘格地面，避免重复地面造成渲染冲突。

也可以单独生成并校验场景：

```bash
ros2 launch m20_nav2_system generate_factory_mujoco_world.launch.py
```

## 地图与代价地图

默认地图位于 `maps/factory/`。PCD 相关工具位于 `scripts/pcd/`，可将实测点云切片、投影为二维占用栅格，并用 A* 做离线连通性检查。Nav2 参数位于 `config/nav2_params.yaml`，MuJoCo/栅格桥参数位于 `config/m20_grid_lidar_simulator.yaml` 和对应 launch 文件。

## 实机接入边界

实机部署不启动以下仿真节点：

```text
m20_mujoco_backend
m20_mujoco_viewer
m20_grid_lidar_simulator
m20_mujoco_odom_adapter
m20_factory_scene_markers
```

替换为 AOS 官方 SDK、真实雷达、真实 LIO 和实机 odom/TF。建议先验证 `/scan`、`/odom` 与完整 TF 树，再以低速、人工急停和命令超时保护接入 Nav2。

## 故障排查

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 topic echo /tf --once
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
```

RViz 不显示机器人时检查 `/robot_description` 和 `map -> odom -> base_link`；地图不显示时检查地图服务是否 active 以及 RViz Fixed Frame 是否为 `map`。详细接口约定见本包 `docs/` 目录。
