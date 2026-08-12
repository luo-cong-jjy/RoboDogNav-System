# SCAN-Planner 首场景整体移植记录

日期：2026-07-27

## 修改目标

统一入口不再用“参数近似 + 定制显示”模拟 SCAN-Planner，而以
`src/third_party/SCAN-Planner` 的首场景运行链作为基准。M20、楼层切换和
巡检任务只通过外围接口接入，不改写 SCAN 的目标、局部规划和可视化话题。

## 差异审计结论

旧入口与第三方首场景存在四项会直接改变效果的差异：

1. 自动任务默认经过二维 A* 子目标链，而原场景直接向 SCAN 下发目标。
2. 局部点云由定制的圆形裁剪器产生，而原场景使用 `pcl_render_node` 的
   360 度射线/遮挡仿真。
3. 规划器、控制器、运动学后端和安全层的速度、几何、地图分辨率不同。
4. 规划器位于 `/m20/navigation` 命名空间并加载定制 RViz，原始
   `/grid_map/*`、`/planning/*`、`/quad_0/cloud` 等显示链被隐藏。

## 实施内容

- 新增 `scan_vendor_planner.yaml`、`scan_vendor_controller.yaml` 和
  `scan_vendor_local_sensing.yaml`。算法参数逐项对应第三方首场景。
- `f1_scan.launch.py` 恢复 SCAN 的根命名空间及标准话题：
  `/move_base_simple/goal`、`/quad_0/cloud`、`/quad_0/lidar_pose`、
  `/grid_map/*`、`/planning/*` 和轨迹 Marker 话题。
- 统一入口加载由原版 `default.rviz` 直接适配 M20 link 名称得到的 RViz
  文件，不再加载仓库定制的 phase-2 显示布局。
- 地图服务器同时发布 `/map_generator/global_cloud`，供原 RViz 和原渲染器
  使用。
- 原 `pcl_render_node` 增加默认关闭的 `allow_global_map_updates` 开关。
  集成配置开启后，楼层切换会清空并重建原渲染器的体素哈希、KD-tree 和
  法线索引；未开启时仍保持第三方原行为。
- 补齐原渲染器 ROS 实体的显式析构顺序，避免整套 launch 结束时由
  file-scope publisher/subscription 晚于 node 析构所引发的退出段错误；
  该修改不进入点云生成或射线模拟算法。
- 新增 `m20_vendor_sensing_state`，只把原渲染器的点云新鲜度转换成楼层事务
  所需的 `LocalSensingState`，不参与点云生成。
- M20 后端和安全监督器使用与原控制器一致的仿真速度包络；实机安全参数
  文件不受影响。
- `m20_scan_planner` 的控制器恢复原 `planning/go2_execution_frozen`
  协议和原控制律。保留的集成扩展只有楼层 reset 服务和到点退出保护。

## 用户运行方式

终端一：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
ros2 launch m20_warehouse_inspection inspection_mission_rviz.launch.py
```

此时 RViz 的 `2D Goal Pose` 直接发布原版 `/move_base_simple/goal`，无需启动
其他导航模式。

终端二（需要自动巡检时）：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
ros2 run m20_warehouse_inspection m20_start_inspection
```

自动巡检和手动目标共用同一个 SCAN 核心，不会另起一套规划器。
