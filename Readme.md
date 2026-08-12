# M20 仓库巡检集成系统

系统源码已收敛到 `src/m20_warehouse_system` 顶层交付目录，唯一用户
入口仍是 ROS 包 `m20_warehouse_inspection`。它组合官方 M20 模型、
SCAN 导航、多层地图、安全与任务状态机、RViz 平面后端和 MuJoCo 动力学后端。

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to m20_warehouse_inspection
```

自由导航：

```bash
source install/setup.bash
ros2 launch m20_warehouse_inspection f1_scan_rviz.launch.py
```

完整模块清单和被隔离的旧包见
[`package_inventory.md`](src/m20_warehouse_system/bringup/m20_warehouse_inspection/docs/package_inventory.md)。
新的顶层目录与仿真/实机边界见
[`m20_warehouse_system/README.md`](src/m20_warehouse_system/README.md)。
