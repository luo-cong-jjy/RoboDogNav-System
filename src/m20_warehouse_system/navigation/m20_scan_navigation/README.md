# M20 SCAN Navigation

本包只负责把仓库系统接到原生 SCAN-Planner 目标链，不再实现第二套全局栅格规划器。
SCAN 的 GridMap、rebound A*、B-spline 优化、闭环控制和原版可视化均位于
`m20_scan_planner`。

运行链路固定为：

```text
/move_base_simple/goal
  -> m20_scan_planner
  -> closed_loop_controller
  -> /m20/navigation/cmd_vel_raw
  -> M20 运动适配与安全层
```

本包保留的集成功能只有：

- 启动原版 CPU local sensing、SCAN planner 和 controller；
- `/m20/navigation/navigate` typed Action，将目标绑定到
  `floor_id + map_generation`；
- 在取消、超时、任务暂停和切层时调用 `/m20/navigation/reset`；
- 碰撞恢复预算耗尽后，停止旧轨迹，等待在线点云守卫重新进入 `CLEAR`，再把同一目标
  交给原生 SCAN 重规划；
- 可选 M20 双向 B-spline 执行参数。该适配只改变下游轨迹执行方向，不改变 SCAN 路径。

地图只有 PCD/JSON 资产。SCAN 从 `/map_generator/global_cloud` 取得当前活动楼层的 PCD，
`pcl_render_node` 生成 `/quad_0/cloud`，SCAN 再发布 `/grid_map/occupancy` 等实时三维点云。
这些 `occupancy` 话题是 `sensor_msgs/PointCloud2`，不是磁盘 PGM/YAML 或
`nav_msgs/OccupancyGrid`。

基础参数读取 `config/scan_vendor_planner.yaml`、`scan_vendor_controller.yaml` 和
`scan_vendor_local_sensing.yaml`。单独启动 `f1_scan.launch.py` 时，规划速度、
`0.25 m` 双圆硬半径、`0.18 m` 圆心偏置、`0.20 m` 优化距离及显示话题都与固定的
`src/third_party/SCAN-Planner` 基线一致。完整 M20 系统随后只叠加
`clearance_m20.yaml` 的机体几何覆盖：硬半径 `0.28 m`、偏置 `0.18 m`、软距离
`0.17 m`。算法和 `0.45 m` 名义单侧包络不变，但硬轴向包络由约
`0.86 × 0.50 m` 增为 `0.92 × 0.56 m`，覆盖已确认的 M20 `0.82 × 0.51 m`
长宽轴向范围。双圆仍是近似模型，不能替代 MuJoCo 接触与真机场地验收。
`grid_map.body_height=0.40 m`在当前 RViz/Action 目标链中不用于从地面抬升目标：
目标高度会锁定首帧`body_pose.z`；它只用于已停用的`initial_path`支撑面接口。
Z向碰撞仍由原生`obstacles_inflation_z_up/down=0.10/0.40 m`和实际点云高度决定，
本轮平面仓库不擅自修改。真机遇到货架横梁、桌面等悬空障碍时必须单独做垂直净空验收。

终点语义固定为 `position_only`。SCAN 读取目标 XY，RViz/Action 中的四元数不是严格
终点 yaw；若业务必须指定终点朝向，应另做可转身区域/分阶段位姿规划，不能要求 M20
在狭窄通道中原地掉头。

启动原生首场景链：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
ros2 launch m20_warehouse_inspection f1_scan_rviz.launch.py
```

本包不直接连接 Gazebo、MuJoCo 或真机 SDK。`scan_native` 默认链为
`cmd_vel_raw -> floor/e-stop safety gate -> cmd_vel_safe`；显式选择 `m20_safe` 时
才增加 `cmd_vel_candidate -> collision guard`。
