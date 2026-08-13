# PCD-only / 原生 SCAN 仿真回归报告

日期：2026-08-11  
平台：Ubuntu 22.04、ROS 2 Humble  
后端：RViz planar kinematic（无 GUI）  
隔离域：`ROS_DOMAIN_ID=179`（导航）、180（退出复验）

## 1. 静态与单元测试

### 1.1 直接源码测试

```bash
python3 -m pytest -q \
  src/m20_inspection_core/test \
  src/m20_scan_navigation/test \
  src/m20_warehouse_inspection/test
```

结果：`157 passed in 7.06s`。

`python3 -m py_compile` 覆盖 navigation gateway、collision guard/policy、safety
supervisor、map assets 和 map server；`git diff --check` 无输出。

### 1.2 colcon 包级测试

| 包 | 结果 |
| --- | --- |
| `m20_scan_navigation` | 41 tests，0 failure |
| `m20_inspection_core` | 86 tests，0 failure |
| `m20_warehouse_inspection` | 212 tests，0 failure，1 skipped |

包级检查包含 pytest、flake8、pep257、CMake/XML lint；首次发现 collision policy 两处
D213 docstring 格式问题，修正后重新执行 `m20_inspection_core` 为 100% 通过。

### 1.3 构建

受影响的 8 包完整构建成功：

```text
m20_warehouse_interfaces
m20_official_description
m20_scan_planner
m20_scan_navigation
m20_inspection_core
m20_locomotion_control
m20_mujoco_backend
m20_warehouse_inspection
```

后续三包增量构建同样成功。

## 2. 运行图检查

启动命令：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_rviz.launch.py \
  use_rviz:=false run_acceptance:=false
```

启动 14 个预期节点，地图服务器报告：

```text
preloaded 2 flat floor maps; active_floor=F1, generation=1,
all_points=289172
```

关键状态：

```text
/m20/map/state: floor=F1, generation=1, ready=true
/grid_map/occupancy: PointCloud2 width=383（采样时）
/m20/control/collision_guard_state: CLEAR
collision local window: 16.0 m, resolution: 0.10 m
height band: [0.09, 1.29] m at body_z=0.59 m
```

话题审计确认不存在：

```text
/m20/map/active_occupancy
/m20/navigation/global_route
/m20/navigation/global_route_state
/m20/control/route_segment_hold
```

`ros2 pkg executables m20_scan_navigation` 只返回：

```text
m20_clearance_report
m20_navigation_gateway
```

## 3. 单目标导航

初始位姿：

```text
x=-37.0, y=0.0, z=0.59
```

发布目标：

```text
x=-35.0, y=0.0, z=0.59
```

SCAN 运行证据：

- trajectory 1：原生规划成功，duration 4.160 s；
- trajectory 2：原生重规划成功，duration 3.200 s；
- trajectory 3：原生重规划成功，duration 1.922 s；
- 三次均 `first_optimize_step_success=1`、`final_plan_success=1`；
- 最终 FSM 从 `EXEC_TRAJ` 进入 `WAIT_TARGET`。

最终位姿：

```text
x=-35.00726731170498, y=0.0, z=0.59
```

XY 终点误差约 `0.0073 m`。结束时：

```text
collision_guard_state: CLEAR
safety_state: NAVIGATION
cmd_vel_raw: zero
```

日志中未出现 `The robot is inside an obstacle`、`A-star error`、
`A-star failed` 或实体碰撞信息。

## 4. 退出复验

首次停止时发现 gateway 在 ROS signal handler 已关闭 context 后再次调用 shutdown，产生
`rcl_shutdown already called`。增加 `if rclpy.ok()` 后在 domain 180 重启并 Ctrl-C：
全部 14 个进程 `finished cleanly`，无 gateway traceback。

## 5. 结论与放行范围

PCD-only 地图、原生 SCAN 目标链、在线点云 collision guard、M20 运动适配和 RViz
平面后端已形成可启动、可规划、可运动、可停车、可干净退出的完整单目标闭环。本轮可
作为后续 MuJoCo 单层多目标复验和 Foxy 实机 F1 台架的源码基线。

本报告不放行真实机器人运动，也不宣称多层任务/电梯或 MuJoCo 11 步已在本轮重新验收。
