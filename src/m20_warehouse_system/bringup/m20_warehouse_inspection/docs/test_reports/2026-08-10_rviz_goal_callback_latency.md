# RViz 点击导航无响应根因与修复记录（2026-08-10）

## 1. 问题现象

完整 MuJoCo、RViz、SCAN 系统启动后，RViz 的 `2D Goal Pose` 消息能够到达
`/move_base_simple/goal`，但机器狗不运动。导航网关状态依次为
`RESETTING_NATIVE`、`RESET_FAILED`，目标随后被安全丢弃。

## 2. 只读诊断结论

问题不是 RViz 没有发布目标，也不是 M20 模型、路径优化或控制 SDK 拒绝目标。
直接原因是导航网关在每个受管 RViz 目标前调用 SCAN 复位服务，但原实现只等待
2 秒。SCAN 的复位回调与点云、占据栅格和 RViz 可视化共用单线程执行器；当
RViz 同时订阅占据与膨胀点云时，复位回调会长时间排队。

诊断阶段测得：

- RViz 开启时，同一复位服务耗时 6.35 秒；
- 仅关闭 RViz、保持其余系统运行时，耗时降到 1.44 秒；
- 绕过网关直接发送内部 SCAN 目标后，规划和运动链能够工作；
- `plan_env` 与 `m20_scan_planner` 当时没有构建类型，实际 C++ 标志不含优化；
- 第三方 SCAN README 明确要求 `-DCMAKE_BUILD_TYPE=Release`。

此外，用户 shell 中的 Fast DDS、Domain 0 设置和遗留同名节点会造成发现不稳定，
但切换至 CycloneDDS、Domain 79 后仍能复现复位超时，因此它不是本次故障的唯一
代码级根因。

## 3. 修改范围

本轮没有修改规划几何、障碍物膨胀、运动学限制或 M20 控制参数。

1. `plan_env` 和 `m20_scan_planner` 在未显式给出构建类型时默认使用 Release，
   与第三方 SCAN 的官方构建要求一致。
2. `plan_env` 增加 `grid_map.visualization_rate_hz` 参数；第三方默认仍为 20 Hz，
   M20 集成配置使用 5 Hz。点云内容不变，只降低全量可视化遍历频率。
3. 导航网关增加 `navigation_reset_timeout_sec`，集成默认值为 10 秒，替代不可配置
   的 2 秒常量，作为单次映射回调延迟的有界兜底。
4. 增加配置契约测试，防止可视化刷新率或复位超时退回故障值。

## 4. 构建与自动测试

构建命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select plan_env m20_scan_planner m20_scan_navigation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --allow-overriding plan_env m20_scan_planner m20_scan_navigation
```

最终确认：

- `plan_env`：`-O3 -DNDEBUG`；
- `m20_scan_planner`：`-O3 -DNDEBUG`；
- `m20_scan_navigation`：41 项 pytest 通过；
- `m20_scan_planner`：2 项 launch test 通过；
- 共 43 项相关自动测试通过。

## 5. 完整 GUI 运行验收

验收环境为 CycloneDDS、Domain 79，完整启动 MuJoCo、官方 M20 SDK、SCAN、RViz、
双层地图、网关和安全链。通过 RViz 相同公共入口发送
`(-35.50, 0.00)` 的 1.5 米安全目标。

结果：

- `RESETTING_NATIVE` 到 SCAN 复位完成约 28 ms；
- 到 `TRACKING_ROUTE` 约 99 ms；
- 到控制器收到首条规划轨迹约 151 ms；
- 未出现 `RESET_FAILED` 或 `SUBGOAL_STALLED`；
- 最终状态为 `SUCCEEDED`，位置误差 0.198 m。

## 6. 启动环境要求

仿真不要依赖全局 `.bashrc` 中可能用于实机的 Fast DDS 设置。当前可靠命令为：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=79
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI=file:///home/virdyn/robodog_nav_system/install/\
m20_warehouse_inspection/share/m20_warehouse_inspection/config/cyclonedds_local.xml
ros2 launch m20_warehouse_inspection inspection_mission_mujoco.launch.py
```

实机若需要 Fast DDS、Domain 0，应在实机专用终端单独设置，不应覆盖仿真终端。

## 7. 后续观察项

模拟激光计算期间仍偶发 2～4 秒的 `ODOM_STALE`，安全监督器会立即置零并在位姿
恢复后解除保持。本轮短目标可以自动恢复并成功完成；该周期性模拟感知负载应作为
后续独立性能优化项，不与本次“点击目标被 2 秒超时丢弃”混为一谈。
