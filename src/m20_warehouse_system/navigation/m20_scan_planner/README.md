# m20_scan_planner

`m20_scan_planner` 是当前 `robodog_nav_system` 工作空间中的 M20 适配版 SCAN-Planner 包。它保留了原 SCAN-Planner 的局部规划主链路，包括 GridMap、路径搜索、B 样条优化、规划状态机和 RViz 可视化，同时把仿真闭环、机器人模型、话题接口和启动入口改成了本工程可直接使用的 M20 版本。

原始三方 ROS 2 移植包在 `src/third_party/SCAN-Planner`，其说明文档见 [../third_party/SCAN-Planner/README.md](../third_party/SCAN-Planner/README.md)。当前工作空间已经登记并可直接启动的包名是 `m20_scan_planner`，所以不要用 `ros2 launch scan_planner ...` 启动这个包。

在仓库集成系统中，本包是唯一被编译和启动的 SCAN 核心；`m20_scan_navigation`
只负责 typed gateway、可选栅格路线和 bringup，不保存另一份 planner/controller。
仓库入口默认把 RViz 目标直接送到 `/m20/navigation/scan_goal`，因而保留原生 SCAN
的局部点云、搜索过程和 B 样条轨迹显示。`/m20/navigation/reset` 是为切层和任务取消
增加的受控清理接口，不改变正常规划算法。

## 和原版相比改了什么

- 包名从 `scan_planner` 改为 `m20_scan_planner`，避免和三方源码包混淆，也方便在主工程里单独维护。
- 机器人描述从 `go2_description` 切换为 M20；当前 `run.launch.py` 直接使用
  `m20_official_description/urdf/m20_official.urdf`。
- 原来的 `go2_kinematic_sim` 和 `go2_gait_publisher` 改成了 `m20_kinematic_sim` 和 `m20_gait_publisher`。
- M20 步态可视化适配了 16 个关节名，包含四个轮关节：`fl/fr/hl/hr_*_joint`。
- 闭环控制器仍订阅 `planning/bspline` 和 `body_pose`，但输出执行速度到 M20 的 `cmd_vel` 路线；仿真时接 `/quad_0/cmd_vel`，实机时接 `/cmd_vel`。
- 规划器和控制器之间的执行冻结话题从 `planning/go2_execution_frozen` 改为 `planning/m20_execution_frozen`。
- `run.launch.py` 集成了 M20 模型、规划主节点、闭环/开环控制器、轻量运动学仿真、步态 JointState 发布、Mockamap/PCD 地图和局部传感器仿真。
- 实机模式默认对接外部 LIO 和传感器话题：`/LIO/odom_vehicle`、`/LIO/odom_imu`、`/LIO/clouds_lidar` 以及 RealSense 对齐深度图话题。
- RViz 配置、关键点记录脚本和测试入口都按 `m20_scan_planner` 包名做了适配。
- 仓库 RViz 基线使用 `m20_official_description` 中由官方
  `src/third_party/deep_robotics_model/M20` 逐项校验的 URDF/mesh，不添加雷达或 IMU。
  本包 `models/m20` 下的文件仅服务于旧的独立 Gazebo 实验入口，不进入仓库 RViz
  模型链路；Gazebo 执行插件仍由对应插件包提供。
- Building Gazebo 入口临时默认使用 `m20_gazebo_legged_no_lidar.urdf`：它不挂载雷达，不用轮子驱动底盘，而是通过 `/cmd_vel` 做四足步态动画和地形跟随位姿更新，便于先复现楼梯/多楼层导航链路。轮足模式选择后续按 [WHEEL_LEG_MODE_DESIGN.md](WHEEL_LEG_MODE_DESIGN.md) 继续完善。

## 目录说明

- `launch/run.launch.py`：主启动入口，负责仿真/实机模式、控制器模式和传感器模式的组合。
- `launch/simulator.launch.py`：仿真地图和局部感知节点入口，由 `run.launch.py` 自动包含。
- `launch/rviz.launch.py`：RViz2 可视化入口。
- `launch/mockamap_m20_gazebo.launch.py`：加载单层随机障碍 Gazebo world 并生成 M20。
- `launch/mockamap_gazebo_scan_planner.launch.py`：加载单层随机障碍 Gazebo world、同步 PCD 地图、M20 和 SCAN-Planner 闭环。
- `launch/building_world.launch.py`：只加载 Gazebo Building world，用于检查建筑模型。
- `launch/building_m20_gazebo.launch.py`：加载 Building world 并生成不带雷达的 M20 Gazebo 模型。
- `launch/building_gazebo_scan_planner.launch.py`：加载 Building world、M20 Gazebo 模型、SCAN-Planner 点云生成链路和闭环控制器。
- `config/planner.yaml`：规划器、地图和优化器参数。
- `config/controllers.yaml`：闭环控制器、开环控制器、M20 运动学仿真和步态可视化参数。
- `config/simulator.yaml`：Mockamap、局部感知和里程计可视化参数。
- `config/keypoints.example.yaml`：`navi_mode:=2` 的路径点参数示例。
- `scripts/generate_single_floor_gazebo_map.py`：生成单层随机障碍的 Gazebo world、同步 PCD 地图和 JSON 元数据。
- `scripts/collada_mesh_to_pcd.py`：把 Collada/DAE 三角网格采样成 PCD 地图。
- `src/scan_planner_node.cpp`：规划主节点。
- `src/closed_loop_controller.cpp`：闭环速度控制器。
- `src/open_loop_controller.cpp`：开环轨迹播放控制器。
- `src/m20_kinematic_sim.cpp`：轻量 M20 运动学仿真。
- `src/m20_gait_publisher.cpp`：M20 JointState 步态可视化发布器。

## 构建

在工作空间根目录执行：

```bash
cd ~/robodog_nav_system
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
sudo apt install libarmadillo-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

colcon build --symlink-install --packages-up-to m20_scan_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
ros2 pkg prefix m20_scan_planner
```

如果最后一条能输出 `~/robodog_nav_system/install/m20_scan_planner` 之类的路径，说明包已经被当前 ROS 2 环境识别。

## 启动步骤

每开一个新终端，都先进入工作空间并加载环境：

```bash
cd ~/robodog_nav_system
source install/setup.bash
```

### 场景一：内置 Mockamap 仿真地图

这是最适合第一次验证的启动方式。它会启动 M20 轻量运动学仿真、Mockamap 点云地图、局部传感器仿真、SCAN-Planner 主节点和闭环控制器。

终端 1：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 launch m20_scan_planner run.launch.py \
  is_real_world:=false \
  navi_mode:=1 \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_gpu:=false
```

终端 2：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 launch m20_scan_planner rviz.launch.py
```

RViz 打开后，使用工具栏里的 `2D Goal Pose` 给 `/move_base_simple/goal` 发送目标点。`navi_mode:=1` 下，规划器会等待这个目标点，然后生成局部 B 样条并由闭环控制器驱动 `/quad_0/cmd_vel`，`m20_kinematic_sim` 会根据速度指令更新 `/quad_0/body_pose`。

默认初始位置在 `run.launch.py` 里是：

```bash
init_x:=-19.0 init_y:=1.0 init_z:=0.3
```

需要换起点时可以直接覆盖：

```bash
ros2 launch m20_scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false \
  init_x:=-10.0 init_y:=0.0 init_z:=0.3
```

#### 场景一 Gazebo 同步版

如果希望 Gazebo 中也出现同一套单层障碍物，用这个入口。它不直接运行 `mockamap_node`，而是使用 `generate_single_floor_gazebo_map.py` 生成的一组同步文件：

```bash
~/robodog_nav_system/src/m20_warehouse_system/navigation/m20_scan_planner/models/mockamap_single_floor/mockamap_single_floor.world
~/robodog_nav_system/src/m20_warehouse_system/navigation/m20_scan_planner/models/mockamap_single_floor/mockamap_single_floor.pcd
~/robodog_nav_system/src/m20_warehouse_system/navigation/m20_scan_planner/models/mockamap_single_floor/mockamap_single_floor.json
```

Gazebo 读取 `.world` 显示实体障碍物；SCAN-Planner/local sensing 读取同源 `.pcd` 发布 `/map_generator/global_cloud` 和 `/quad_0/cloud`，所以 Gazebo 与 RViz 看到的是同一批障碍物。

启动：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 launch m20_scan_planner mockamap_gazebo_scan_planner.launch.py \
  gui:=true \
  use_rviz:=true \
  navi_mode:=1 \
  sensor_type:=lidar \
  use_gpu:=false
```

启动后在 RViz 中用 `2D Goal Pose` 发布目标点。Gazebo 负责显示单层障碍环境和 M20，规划侧仍使用 SCAN-Planner 的 PCD/局部点云链路，不依赖机器人 URDF 上的 Gazebo 雷达。

默认地图参数接近场景一：`seed=127`、地图大小 `40m x 40m x 5m`、障碍高度 `2m`。为了让 WSL2 下 Gazebo GUI 保持可用，默认实体障碍数量设为 `180`。如果要重新生成地图，可以运行：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 run m20_scan_planner generate_single_floor_gazebo_map.py \
  --output-dir src/m20_warehouse_system/navigation/m20_scan_planner/models/mockamap_single_floor \
  --seed 127 \
  --obstacle-count 180

colcon build --symlink-install --packages-select m20_scan_planner
```

如果要临时只检查 Gazebo 世界和 M20，不启动规划器：

```bash
ros2 launch m20_scan_planner mockamap_m20_gazebo.launch.py gui:=true
```

### 场景二：读取已有 PCD 地图

这个模式不使用 Mockamap 生成地图，而是读取已有 PCD 文件。当前工程里可直接使用三方包自带的示例地图：

```bash
~/robodog_nav_system/src/third_party/SCAN-Planner/map.pcd
```

终端 1：

```bash
cd ~/robodog_nav_system
source install/setup.bash

PCD_MAP="$(pwd)/src/third_party/SCAN-Planner/map.pcd"

ros2 launch m20_scan_planner run.launch.py \
  is_real_world:=false \
  navi_mode:=1 \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_gpu:=false \
  use_pcd_map:=true \
  pcd_map_file:="$PCD_MAP"
```

终端 2：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 launch m20_scan_planner rviz.launch.py
```

PCD 模式下，`map_generator/map_pub` 会读取 `pcd_map_file` 并发布到 `/map_generator/global_cloud`，局部感知节点再从全局地图中模拟 M20 当前视野内的点云。

### Gazebo 建筑世界预览

`models/1_Building` 中提供了一个 Gazebo Classic 建筑世界模型。这个入口只用于先验证 Gazebo 中能否加载建筑场景，还没有接入 SCAN-Planner 的规划闭环或 M20 Gazebo 执行链路。

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 launch m20_scan_planner building_world.launch.py gui:=true
```

如果只想在后台验证 world 是否能加载，可以关闭 GUI：

```bash
ros2 launch m20_scan_planner building_world.launch.py gui:=false
```

### Gazebo 建筑世界 + M20

确认建筑世界能加载后，可以启动 Building world 并放入不带雷达的 M20 Gazebo 模型。该入口默认使用 `models/m20/m20_gazebo_legged_no_lidar.urdf`，模型网格为本包内的 M20 隔离副本，不挂载 Gazebo 雷达；`/cmd_vel` 会进入临时四足运动学插件，插件发布 `/odom` 和 `/joint_states`，同时根据当前位置附近的建筑碰撞面做地形跟随，让模型先能过楼梯/楼层高度变化。点云仍由 SCAN-Planner 的本地感知/地图采样链路提供，不从 M20 URDF 上的 ray sensor 产生。

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 launch m20_scan_planner building_m20_gazebo.launch.py gui:=true
```

默认会在 `x=-5.0 y=7.0 z=0.59 yaw=0.0` 生成 M20，可以覆盖初始位姿：

```bash
ros2 launch m20_scan_planner building_m20_gazebo.launch.py \
  gui:=true x:=-5.0 y:=7.0 z:=0.59 yaw:=0.0
```

启动后可以在另一个终端发布速度指令做基础移动测试：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

保持发布 2 到 3 秒后按 `Ctrl-C` 停止。如果 Gazebo 顶部工具栏处于暂停状态，先点播放按钮让仿真时间流动。

如果想直接用 Gazebo 判断模型是否真的移动，可以在发布 `/cmd_vel` 前后各读一次模型位姿：

```bash
GAZEBO_MASTER_URI=http://127.0.0.1:11345 GAZEBO_IP=127.0.0.1 \
  gz model -m m20 -p
```

如果终端一直显示 `publishing #...` 但 Gazebo 里不动，优先检查三件事：

- Gazebo 顶部工具栏是否处于暂停状态，底部仿真时间是否在走。
- 启动 Gazebo 和发布 `/cmd_vel` 的两个终端是否都执行了 `source install/setup.bash`。
- 两个终端的 `ROS_DOMAIN_ID` 是否一致；若不确定，运行 `echo $ROS_DOMAIN_ID`，保持两边相同或都为空。

`Ctrl-C` 后偶尔出现 `failed to shutdown: rcl_shutdown already called` 是 ROS 2 CLI 退出清理提示，不代表速度指令发送失败。

常用 Gazebo 话题：

- `/cmd_vel`：M20 四轮 Gazebo 插件的速度输入
- `/odom`：M20 四轮 Gazebo 插件发布的里程计

这个入口不会发布 `/LIDAR/POINTS`，也不会启动 `pointcloud_to_laserscan`。建筑导航点云后续应接 SCAN-Planner 自己的点云生成方法，而不是挂在 M20 URDF 上的 Gazebo ray sensor。

如果要临时切回轮式 Gazebo 执行器，可以显式指定原来的无雷达轮式 URDF，并打开站立关节发布器：

```bash
cd ~/robodog_nav_system
source install/setup.bash

WHEEL_URDF="$(ros2 pkg prefix m20_scan_planner)/share/m20_scan_planner/models/m20/m20_gazebo_official_no_lidar.urdf"

ros2 launch m20_scan_planner building_m20_gazebo.launch.py \
  gui:=true \
  gazebo_robot_urdf:="$WHEEL_URDF" \
  publish_standing_joints:=true
```

注意：当前默认四足模式是临时运动学执行器，不是足端受力、触地检测和全身控制闭环。它适合先验证 SCAN-Planner、Building PCD、Gazebo world 和多楼层目标的复现流程；真正的“直线/大曲率用轮、狭窄/楼梯切腿”的模式判断和执行切换在 `WHEEL_LEG_MODE_DESIGN.md` 中继续实现。

WSL2 下如果看到 `Unable to open audio device[default]`，这是音频设备不可用提示，不影响 Gazebo 世界、M20 模型或导航测试。当前 launch 已固定 `GAZEBO_IP=127.0.0.1`，用于避免 Gazebo 在 WSL2 中把服务地址发布到不可达网卡导致 spawn/service 返回卡住。

### Gazebo 建筑世界 + M20 + SCAN-Planner 闭环

这是把当前 Gazebo M20 执行链路和 SCAN-Planner 接起来的入口。Gazebo 负责加载 Building world、生成 M20、发布 `/odom` 并接收 `/cmd_vel`；SCAN-Planner 侧默认读取 Building mesh 采样出的 PCD 地图，再由 `local_sensing_node` 根据 Gazebo 的 `/odom` 裁剪出实时局部点云 `/quad_0/cloud`。这里仍然不使用 Gazebo 机器人雷达，也不会发布 `/LIDAR/POINTS`。

默认使用的 Building PCD 是：

```bash
~/robodog_nav_system/src/m20_warehouse_system/navigation/m20_scan_planner/models/1_Building/Building_surface_0p10.pcd
```

它由 `Building.dae` 按约 0.1m 表面分辨率采样生成，并应用了 `model.sdf` 中 Building link 的 `z=-0.1` 偏移，所以和 Gazebo 里的 Building 位置对齐。直接启动：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 launch m20_scan_planner building_gazebo_scan_planner.launch.py \
  gui:=true \
  use_rviz:=true \
  navi_mode:=1 \
  sensor_type:=lidar \
  use_gpu:=false \
  use_pcd_map:=true
```

启动后会同时打开 Gazebo 和 RViz。Gazebo 里能看到 Building world 和 M20；RViz 里看 `/quad_0/cloud`、规划轨迹和 M20 位姿。用 RViz 工具栏的 `2D Goal Pose` 给 `/move_base_simple/goal` 发目标点后，规划器会发布 `/planning/bspline`，闭环控制器会把速度发到 `/cmd_vel`，Gazebo 中的 M20 应该开始移动。

#### 跨楼层支撑面全局路径模式

当前包已经加入第一版轻量 `traversability_global_planner.py`，用于验证 PCT/TravExplorer 风格的几何可通行全局层。它会从 Building PCD 中提取可站立支撑面，把楼层、楼梯和平台连成支撑面图，再把 RViz 目标点转换成贴地的 `/initial_path`。SCAN-Planner 需要切到 `navi_mode:=3` 来订阅这条路径：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 launch m20_scan_planner building_gazebo_scan_planner.launch.py \
  gui:=true \
  use_rviz:=true \
  navi_mode:=3 \
  sensor_type:=lidar \
  use_gpu:=false \
  use_pcd_map:=true \
  use_traversability_global_planner:=true
```

这个 Building 入口会同时覆盖 SCAN-Planner 的机身高度和竖向碰撞膨胀：

```bash
traversability_body_height:=0.59
building_obstacles_inflation_z_up:=0.15
building_obstacles_inflation_z_down:=0.20
traversability_waypoint_spacing:=0.80
traversability_vertical_waypoint_spacing:=0.25
traversability_max_output_waypoints:=28
traversability_max_goal_candidates:=16
traversability_goal_layer_policy:=nearest_to_start_height
building_sliding_map_size_z:=10.0
building_local_update_range_z:=5.0
```

原因是 `/initial_path` 由支撑面高度发布，`scan_planner_node` 会再加 `grid_map.body_height` 转换成机身中心高度。M20 在 Gazebo 中站立时机身中心约为 `0.59m`，如果沿用原版 `0.40m`，局部优化器容易认为机身碰撞体已经压进地面或楼梯支撑面，表现为收到目标但不动，并在日志里出现 `The robot is inside an obstacle` 或 `A-star failed`。

后几个 `traversability_*` 参数用于减少点击目标后的等待时间：支撑面全局层默认优先选择接近当前高度的可达目标层，只在目标附近尝试有限个高度候选，并把输出给 SCAN-Planner 的 `/initial_path` 压缩成较少的关键点。节点日志中的 `plan_time=...s` 可以用来判断耗时是否还集中在全局支撑面 A*。如果要强制测试高楼层，再启动时改成 `traversability_goal_layer_policy:=highest`，或者用 RViz 的 `Publish Point` 点选带 z 高度的点云位置。

如果当前目标是先验证“高层支撑面路径 + Gazebo 中 M20 是否能沿路走起来”，可以启用直接支撑路径跟踪模式：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 launch m20_scan_planner building_gazebo_scan_planner.launch.py \
  gui:=true \
  use_rviz:=true \
  navi_mode:=3 \
  sensor_type:=lidar \
  use_gpu:=false \
  use_pcd_map:=true \
  use_traversability_global_planner:=true \
  use_support_path_follower:=true \
  traversability_goal_layer_policy:=highest
```

这个模式会绕过 `scan_planner_node` 和 `closed_loop_controller`，由 `support_path_follower.py` 直接跟踪 `/initial_path` 并发布 `/cmd_vel`。它不做局部 B 样条避障，适合当前阶段验证跨楼层支撑面图和 Gazebo 机器人响应；后续正式方案仍需要把“楼板/楼梯是可支撑面，而墙体/栏杆是障碍物”的语义接入 SCAN 局部层。

使用方式：

- RViz 的 `2D Goal Pose` 仍发到 `/move_base_simple/goal`。如果同一个 `(x, y)` 附近只有一个可通行高度，节点会自动吸附到该支撑面。
- 如果 `(x, y)` 上下有多层重叠，默认 `traversability_goal_layer_policy:=nearest_to_start_height` 会优先选择接近当前高度的可达目标层，响应更快，适合日常同层/近层测试。
- 如果想临时强制测试高楼层，可以启动时加 `traversability_goal_layer_policy:=highest`。
- 如果想给一个明确 3D 高度提示，可以在 RViz 用 `Publish Point` 点选点云/支撑面，节点会订阅 `/clicked_point` 并按该点的 `z` 匹配目标支撑面。

离线检查 PCD 支撑面图是否能连通，可以不启动 Gazebo，直接跑：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 run m20_scan_planner traversability_global_planner.py \
  --no-ros \
  --pcd src/m20_warehouse_system/navigation/m20_scan_planner/models/1_Building/Building_surface_0p10.pcd \
  --start -5.0 7.0 0.59 \
  --goal 2.0 -3.0 4.4
```

这版仍是几何验证版，不包含语义楼梯识别、主动补观测、ESDF 距离代价或真正足端可达性评估。它的作用是先解决“跨楼层目标不能直接飞线、目标高度不应完全靠手点”的全局层问题；后续再把支撑面图、局部 B 样条高度约束和 `WHEEL_LEG_MODE_DESIGN.md` 的轮足模式选择接起来。

这个入口默认启用闭环控制器的前向优先模式：

```bash
controller_forward_only:=true
controller_heading_error_threshold:=0.35
controller_max_forward_lateral_speed:=0.05
```

含义是：新目标在机体后方或转角较大时，先冻结轨迹时间并原地转向；正常跟踪时禁止倒退，横向速度只保留很小余量。这样 Gazebo 里的 M20 不会因为上一段导航留下的朝向不同而出现一会儿正走、一会儿倒着走的观感。若要临时恢复原始全向跟踪行为，可以加 `controller_forward_only:=false`。

如果修改了 `Building.dae` 或换了别的 DAE，可以重新生成 PCD：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 run m20_scan_planner collada_mesh_to_pcd.py \
  src/m20_warehouse_system/navigation/m20_scan_planner/models/1_Building/Building.dae \
  src/m20_warehouse_system/navigation/m20_scan_planner/models/1_Building/Building_surface_0p10.pcd \
  --spacing 0.08 \
  --voxel-size 0.10 \
  --translation 0 0 -0.1
```

如果要临时读取其他 PCD 地图，覆盖 `pcd_map_file`：

```bash
cd ~/robodog_nav_system
source install/setup.bash

PCD_MAP="/path/to/your/map.pcd"

ros2 launch m20_scan_planner building_gazebo_scan_planner.launch.py \
  gui:=true \
  use_rviz:=true \
  navi_mode:=1 \
  sensor_type:=lidar \
  use_gpu:=false \
  use_pcd_map:=true \
  pcd_map_file:="$PCD_MAP"
```

如果只是想退回随机 Mockamap 链路验证，可以显式关闭 PCD：

```bash
ros2 launch m20_scan_planner building_gazebo_scan_planner.launch.py \
  gui:=true use_rviz:=true use_pcd_map:=false
```

注意：当前 Building PCD 是从静态建筑 mesh 表面采样出来的全局点云，能让 planner/local_sensing 看到和 Gazebo Building 对齐的楼体几何。后续要做真正多楼层导航，还需要继续处理楼层连通、楼梯/坡道可通行区域、目标点高度和轮足运动模式切换。

常用检查命令：

```bash
ros2 topic echo --once /odom
ros2 topic hz /quad_0/cloud
ros2 topic hz /planning/bspline
ros2 topic echo --once /cmd_vel
```

如果 Gazebo 里 M20 不动，先确认 Gazebo 没有暂停，并确认 `/cmd_vel` 有速度：

```bash
ros2 topic echo /cmd_vel
```

如果 `/quad_0/cloud` 没有频率，重点检查 `/odom` 是否已经发布，以及 `mockamap_node` 或 `map_pub` 是否在发布 `/map_generator/global_cloud`。

如果 RViz 的 M20 `RobotModel` 显示 `No transform from [base_link] to [world]`，说明缺少机器人根节点到世界系的 TF。`building_gazebo_scan_planner.launch.py` 会发布静态 `world -> odom`，并把 Gazebo `/odom` 通过 `odom_visualization` 发布成 `odom -> base_link`，所以遇到这个问题时先重启当前 launch，并确认 TF 链存在：

```bash
ros2 run tf2_ros tf2_echo world odom
ros2 run tf2_ros tf2_echo world base_link
```

如果 Gazebo 提示 `Unable to start server[bind: Address already in use]`，说明 `11345` 上已经有一个 Gazebo master。可以先关闭旧 Gazebo，或临时换端口：

```bash
ros2 launch m20_scan_planner building_gazebo_scan_planner.launch.py \
  gui:=true use_rviz:=true gazebo_master_uri:=http://127.0.0.1:11346
```

### 预设路径点模式

`navi_mode:=2` 会从参数文件读取 `fsm.waypoints`，适合复现固定路线：

```bash
cd ~/robodog_nav_system
source install/setup.bash

KEYPOINTS="$(ros2 pkg prefix m20_scan_planner)/share/m20_scan_planner/config/keypoints.example.yaml"

ros2 launch m20_scan_planner run.launch.py \
  is_real_world:=false \
  navi_mode:=2 \
  keypoints_file:="$KEYPOINTS" \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_gpu:=false
```

也可以用关键点记录器从里程计中录制路径点：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 run m20_scan_planner keypoint_recorder.py \
  --odom /quad_0/body_pose \
  --output /tmp/m20_keypoints.yaml
```

录制完成后，把 `keypoints_file:=/tmp/m20_keypoints.yaml` 传给 `run.launch.py`。

### 外部全局路径模式

`navi_mode:=3` 订阅 `/initial_path`，适合接上层全局规划器。启动方式：

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 launch m20_scan_planner run.launch.py \
  is_real_world:=false \
  navi_mode:=3 \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_gpu:=false
```

此时需要外部节点发布 `nav_msgs/msg/Path` 到 `/initial_path`。

## 实机模式

实机模式不会启动 Mockamap、PCD 地图、局部传感器仿真、`m20_kinematic_sim` 或 `m20_gait_publisher`。它只启动规划器、M20 `robot_state_publisher` 和控制器，并默认对接外部 LIO、传感器和底盘执行链路。

```bash
cd ~/robodog_nav_system
source install/setup.bash

ros2 launch m20_scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=1 \
  sensor_type:=lidar \
  controller_mode:=closed_loop
```

默认实机话题映射如下：

- `body_pose` -> `/LIO/odom_vehicle`
- `sensor_pose` -> `/LIO/odom_imu`
- `cloud` -> `/LIO/clouds_lidar`
- `depth` -> `/camera/aligned_depth_to_color/image_raw`
- `cmd_vel` -> `/cmd_vel`

如果现场驱动话题不同，优先修改 `launch/run.launch.py` 中 `is_real` 分支的 remapping。

## 常用参数

- `is_real_world`：`false` 为内置仿真，`true` 为实机接口。
- `navi_mode`：`1` 使用 RViz 目标点，`2` 使用参数文件路径点，`3` 订阅 `/initial_path`。
- `sensor_type`：`lidar` 或 `depth`。
- `controller_mode`：`closed_loop` 输出 `cmd_vel`，`open_loop` 直接播放规划轨迹并发布 `body_pose`。
- `use_gpu`：`false` 使用 `pcl_render_node`，`true` 使用 OpenGL 后端 `opengl_render_node`。
- `use_pcd_map`：`false` 使用 Mockamap，`true` 读取 `pcd_map_file`。
- `pcd_map_file`：PCD 地图绝对路径或当前 shell 可展开出的路径。
- `map_size_x/y/z`：Mockamap 和局部感知地图范围。
- `init_x/y/z`：仿真模式下 M20 初始位置。
- `building_gazebo_scan_planner.launch.py` 额外常用参数：`gui`、`use_rviz`、`x/y/z/yaw`、`spawn_delay`、`gazebo_robot_urdf`、`publish_standing_joints`、`gazebo_master_uri`、`controller_forward_only`、`controller_heading_error_threshold`、`controller_max_forward_lateral_speed`。该入口默认 `use_pcd_map:=true`，默认 PCD 为 `models/1_Building/Building_surface_0p10.pcd`，默认 `map_size_z:=8.0`，默认 Gazebo 执行为临时四足地形跟随模式。

## 主要话题

- 输入：`body_pose`、`sensor_pose`、`cloud`、`depth`、`/move_base_simple/goal`、`/initial_path`
- 规划输出：`planning/bspline`、`planning/data_display`
- 控制输出：仿真为 `/quad_0/cmd_vel`，实机为 `/cmd_vel`
- 仿真里程计：`/quad_0/body_pose`
- 仿真点云：`/map_generator/global_cloud`、`/quad_0/cloud`
- Gazebo M20：`/odom`、`/cmd_vel`
- 可视化：`/quad_0/path`、`/quad_0/robot`、`/grid_map/sliding_map_bbox`

## 常见问题

如果出现：

```text
Package 'scan_planner' not found
```

原因是当前主工程登记的包名是 `m20_scan_planner`。使用：

```bash
ros2 launch m20_scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false
```

如果出现：

```text
use_pcd_map=true requires pcd_map_file to reference an existing PCD file
```

说明 `pcd_map_file` 路径不存在。建议从工作空间根目录设置：

```bash
PCD_MAP="$(pwd)/src/third_party/SCAN-Planner/map.pcd"
test -f "$PCD_MAP" && echo ok
```

如果 RViz 中没有动起来，先确认主 launch 已经收到目标点，并检查这些话题是否存在：

```bash
ros2 topic list | grep -E 'move_base_simple|planning/bspline|quad_0/body_pose|quad_0/cmd_vel'
```

## 测试

```bash
cd ~/robodog_nav_system
source install/setup.bash

colcon test --packages-select m20_scan_planner
colcon test-result --verbose
```
