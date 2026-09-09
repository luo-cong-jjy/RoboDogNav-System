# 仿真链路验收与变更记录

本文件只记录可能影响 Gazebo、MuJoCo 或二者共享 Nav2 部分的修改。修改前必须标明影响域。

### 2026-09-02 Gazebo/MuJoCo 项目资源完全隔离

- `GAZEBO_ONLY`：新增 Gazebo 专属 world、2D/3D 机器人 URDF、Nav2/slam_toolbox 参数、
  RViz 配置、巡检任务配置，以及 `*_gazebo` 的键盘、站立关节、目标点时间戳和点云转扫描脚本。
- `MUJOCO_ONLY`：新增 `nav2_params_mujoco.yaml`、`nav2_sandbox_mujoco.rviz`、
  MuJoCo 巡检任务配置和任务入口；原 MuJoCo 传感器桥、官方 SDK 和 MJCF 生成链保持不变。
- `SHARED_NAV2`：只共享外部 Nav2/slam_toolbox 算法和 ROS 接口，不共享项目自有后端文件。
- Gazebo 建图入口使用 `factory_environment_gazebo_mapping.world`（无动态障碍）；Gazebo
  保存地图/动态避障使用 `factory_environment_gazebo.world`。MuJoCo 始终使用
  `factory_environment_mujoco.world`。

### 2026-09-01 Gazebo 建图世界静态化

- `GAZEBO_ONLY`：新增 `factory_environment_mapping.world`，保留工厂地面、
  围墙和工作台，移除全部动态工人及运动插件。
- 仅 `factory_slam_navigation.launch.py` 默认改用该纯静态世界；保存地图导航、
  动态避障验证和 MuJoCo 链路继续使用各自原有场景。

## 影响域

- `SHARED_NAV2`：两条链路共享的 Nav2 参数、规划器、控制器或任务节点。
- `GAZEBO_ONLY`：Gazebo 世界、传感器、四轮插件、仿真时间及其 launch。
- `MUJOCO_ONLY`：MuJoCo 后端、SDK 适配、代码雷达/里程计及其 launch。

### 2026-09-01 Gazebo 建图 RViz 参数隔离

- `GAZEBO_ONLY`：将 Gazebo 传感器子 launch 的 RViz 参数改名为
  `gazebo_use_rviz`，避免与外层建图/导航 launch 的 `use_rviz` 同名覆盖。
- `factory_slam_navigation.launch.py` 现在默认会启动一个 RViz；MuJoCo 和实机
  launch 未修改。

## 2026-09-01

### MUJOCO_ONLY：恢复独立静态导航坐标链

- `m20_mujoco_navigation.launch.py` 默认加载 `factory_world_map.yaml`。
- 新增 `m20_mujoco_static_navigation.launch.py`。
- MuJoCo 链路不启动 `slam_toolbox` 或 AMCL；由固定 `map -> odom` 配合
  MuJoCo `odom -> base_link`。
- Gazebo 的实时建图和保存地图导航 launch 未修改。

### MUJOCO_ONLY：修复启动后全进程退出

- `m20_mujoco_static_navigation.launch.py` 向 Nav2 传入 Python 可解析的
  `use_composition=False`，修复 `name 'false' is not defined`。

### SHARED_NAV2：巡检任务时间源可配置

- `factory_inspection_mission.launch.py` 新增 `use_sim_time` 参数。
- 默认 `false`，用于 MuJoCo/实机；Gazebo 巡检必须传 `use_sim_time:=true`。
- 任务路径和 NavigateToPose action 接口未改变。

### Gazebo 验收

- `m20_nav2_system` 导航包编译通过。
- `factory_slam_navigation.launch.py` 能展开并启动 Gazebo 传感器层、
  `slam_toolbox` 和 Nav2 navigation servers；RPP、NavFn、局部/全局代价地图
  均完成配置。
- 启动清单中未出现 `m20_nav2_backend`、`rl_deploy_cmdvel`、
  `m20_grid_lidar_simulator` 或 `m20_mujoco_odom_adapter`，确认未混入 MuJoCo 链路。
- 自动化环境禁止创建 Gazebo/ROS UDP 接口，`gzserver` 以
  `Unable to get local interface addresses` 退出，因此本环境无法完成 `/clock`、
  `/scan`、`odom -> base_link` 和实际运动的运行时验收；需在用户桌面 ROS 环境复验。
- 本轮未修改 Gazebo 或 MuJoCo 运行逻辑。
### 2026-09-01 Gazebo 模型资源解析

- Gazebo 专用 `m20_gazebo_combined_2d_scan.urdf` 的网格资源由
  `package://` 改为 `model://m20_nav2_system/...`。
- 原因：Gazebo 日志显示实体 `m20_nav_proxy` 已成功生成，但部分环境中
  `package://` 网格解析不稳定，可能只看到碰撞体而看不到机器狗外观。
- 影响范围：仅 Gazebo 可视模型加载；RViz 使用的 `M20_nav_visual.urdf`、
  MuJoCo 模型、TF、`/scan`、`/odom` 和 Nav2 规划链路均未修改。
- 验证：已在工作空间中完成 `m20_nav2_system` 编译，安装后的 URDF 已确认
  使用 `model://` URI，且安装目录包含全部 STL 网格。

### 2026-09-01 Gazebo 建图键盘控制集成

- `GAZEBO_ONLY`：`factory_slam_navigation.launch.py` 新增并默认启用
  `m20_keyboard_teleop`，建图入口启动后可直接用键盘驱动四轮简化模型。
- 通过 `use_keyboard:=false` 可关闭键盘节点，避免它与 Nav2 自主控制同时发布
  `/cmd_vel`；保存地图后的自主导航和巡检入口保持不变。

### 2026-09-01 Gazebo 保存地图导航固定使用动态障碍场景

- `GAZEBO_ONLY`：`factory_saved_map_navigation.launch.py` 显式向 Gazebo
  传入动态场景文件，不再依赖底层传感器 launch 的默认世界。
- 该世界在与建图世界相同的静态墙体、围栏和工作台基础上，额外运行 5 个
  参与 `/scan` 的移动圆柱障碍，用于验证 AMCL + Nav2 的实时动态避障。
- `factory_slam_navigation.launch.py` 仍只使用
  `factory_environment_mapping.world`，因此动态障碍不会被写入静态地图；MuJoCo
  链路未修改。

### 2026-09-01 09:32 MuJoCo 验收基线恢复与场景隔离

- `MUJOCO_ONLY`：恢复 09:32:20 至 10:00:03 完整运行时使用的动态障碍基线到
  `factory_environment_mujoco.world`。MuJoCo 的 MJCF 生成、代码雷达和 RViz
  场景标记均只读取这个私有 world。
- 该基线保留历史轨迹的原始 `1.0 m/s` SDF 时间轴，并通过
  `dynamic_speed_scale=0.35` 以 `0.35 m/s` 运行；代码雷达和场景标记统一以
  `time.time()` 推进，确保真实动态圆柱、预测轨迹和 MuJoCo Viewer 使用同一相位。
- 该 MuJoCo world 中的 Gazebo `actor` 仅是历史视觉描述；MJCF 转换器只转换
  具有碰撞几何的 model，因此 MuJoCo Viewer 中只会出现可被代码雷达感知的
  动态圆柱，不会出现 Gazebo 人物模型。
- `GAZEBO_ONLY`：`factory_environment.world` 与 MuJoCo 基线不再共用文件；
  Gazebo 建图、保存地图导航和动态障碍实验可以独立调整，不会悄然改变 MuJoCo
  已验收的运动验证链路。
- 该基线没有 `dynamic_time_origin` 或项目侧 `/cmd_vel` 覆盖。它保留独立的
  `/scan_predicted`：预测窗口 `7.0 s`、最大窗口 `8.0 s`、提前量 `3.5 s`、
  尾部 `0.25 s`、8 个采样点；对应的
  `/m20/factory/predicted_obstacles` 标记在 RViz 默认显示。预测扫描只参与全局
  代价地图的提前重规划，真实 `/scan` 仍是局部控制的唯一障碍输入。

### 2026-09-02 MuJoCo 关闭日志与预测扫描性能修复

- `MUJOCO_ONLY`：`m20_grid_lidar_simulator` 和
  `m20_factory_scene_markers` 捕获正常的 Ctrl-C，关闭时不再打印
  `KeyboardInterrupt` traceback 或被 launch 标记为异常退出。
- `MUJOCO_ONLY`：`m20_mujoco_odom_adapter` 对 rclpy 在 SIGINT 期间销毁订阅
  的 `RuntimeError: Unable to convert call argument` 竞态进行干净收尾；运行中
  的真实转换错误仍会记录并抛出。
- `MUJOCO_ONLY`：预测 `/scan_predicted` 复用同一帧实际 `/scan` 的静态地图
  射线结果，只额外计算未来动态障碍命中，避免每帧重复遍历占据栅格，降低
  `bt_navigator` 的偶发 `Behavior Tree tick rate exceeded` 风险；预测话题和
  代价地图连接未改变。

### 2026-09-02 Gazebo 四轮掉头控制

- `GAZEBO_ONLY`：`nav2_params_gazebo.yaml` 开启
  `use_rotate_to_heading`。当路径切线与当前朝向相差超过 45° 时，RPP
  先发布零线速度和角速度，驱动四轮差速模型原地对齐，再恢复前进跟踪。
- 四轮插件本身支持该运动：`linear.x = 0` 时左右轮目标速度方向相反，
  因此可绕机身中心旋转；MuJoCo 控制器参数未修改。
- 目的：减少通道端点掉头时的大半径曲线，避免机身进入静态障碍膨胀区。

### 2026-09-02 Gazebo 实测动态障碍跟踪

- `GAZEBO_ONLY`：新增 `m20_gazebo_dynamic_obstacle_tracker`。它只订阅
  Gazebo 实际发布的 `/scan`、保存地图 `/map`、`/odom` 和 TF；不读取 world/SDF
  中的 actor、waypoint 或速度轨迹，因此没有 MuJoCo 链路的先验答案。
- 跟踪器从实测扫描中剔除与静态地图一致的回波，对剩余聚类做数据关联和恒速估计，
  发布 `/scan_predicted_gazebo` 及两个 Gazebo 专属 RViz 标记话题：
  `/m20/factory/gazebo_dynamic_obstacles`、
  `/m20/factory/gazebo_predicted_obstacles`。
- Gazebo 保存地图导航默认启用跟踪器；全局代价地图使用实测 `/scan` 加预测扫描，
  局部代价地图仍只使用实测 `/scan`，避免预测区域导致控制器提前停死。
- `factory_slam_navigation.launch.py` 建图入口不启用跟踪器，建图世界不包含动态
  worker，静态地图不会把动态障碍写入地图。
- `MUJOCO_ONLY`：`m20_grid_lidar_simulator`、`/scan_predicted` 和原有 MuJoCo
  标记话题保持不变；两套预测节点、话题、参数和 world 文件互不复用。
