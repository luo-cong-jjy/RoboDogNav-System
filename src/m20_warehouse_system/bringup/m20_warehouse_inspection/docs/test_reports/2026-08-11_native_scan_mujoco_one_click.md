# 原生 SCAN 与 MuJoCo 一键闭环复验

日期：2026-08-11  
平台：Ubuntu 22.04、ROS 2 Humble  
隔离域：`ROS_DOMAIN_ID=181`  
入口：`inspection_mission_mujoco.launch.py`

## 1. 与本地 SCAN-Planner 基线的关系

对比基线为 `src/third_party/SCAN-Planner`，不是网络上的其他分支。

| 层级 | 当前实现 | 与基线的关系 |
| --- | --- | --- |
| GridMap、A*、B-spline 优化、轨迹消息 | `plan_env`、`path_searching`、`bspline_opt`、`traj_utils` | 直接链接本地 third-party 包，不是重写实现 |
| planner manager | `m20_scan_planner/src/planner_manager.cpp` | 去除标记的短轨迹动态可行性计时扩展并规范化后，与本地基线源码一致 |
| planner 参数 | `scan_vendor_planner.yaml` | 基线全部键值逐项一致；只增加到点容差和可视化频率两个集成参数 |
| closed-loop 参数 | `scan_vendor_controller.yaml` | 基线 closed-loop 全部键值一致 |
| FSM | `m20_scan_planner/src/scan_replan_fsm.cpp` | 保留原生状态机，增加近目标结束、地图切换 reset 和外部安全 hold |
| 控制器 | `m20_scan_planner/src/closed_loop_controller.cpp` | 保留原控制律，增加 hold 与 M20 前进/倒车方向选择 |
| 仿真执行模型 | MuJoCo M20 + 官方 ONNX 策略 | 有意替换原包理想 Go2 质点/运动学模拟器，不追求同一运动结果 |
| 地图/任务/安全 | PCD 楼层、generation、collision guard、mission Actions | 项目外围新增能力，不修改 SCAN 的全局/局部轨迹算法 |

`grid_map.visualization_rate_hz` 在当前主链为 5 Hz，原包源码默认值为 20 Hz；话题、
点云内容和规划可视化语义不变。降低频率是为了避免本机单线程规划节点反复遍历大体素
可视化缓存而阻塞目标回调。

源码契约复验：

```bash
python3 -m pytest -q \
  src/m20_scan_navigation/test/test_navigation_contract.py
```

结果：`20 passed in 0.05s`。测试包含 planner manager 规范化源码一致性、原生 planner
参数逐键一致性和 closed-loop 参数逐键一致性。

## 2. 一键冷启动

测试命令：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_mujoco.launch.py \
  use_rviz:=false \
  use_mujoco_viewer:=false \
  run_acceptance:=false
```

只关闭 RViz 和 MuJoCo 图形窗口；以下生产链均实际启动：

```text
双层 PCD map server -> local sensing -> SCAN GridMap/planner/controller
-> M20 navigation adapter -> collision guard -> safety supervisor
-> locomotion manager -> 官方 rl_deploy_cmdvel/ONNX policy
-> MuJoCo M20 dynamics -> odometry/TF feedback
```

启动 16 个进程。关键冷启动状态：

```text
MuJoCo official SDK motor command detected; startup hold released
MuJoCo M20 backend ready: official SDK stand-up is stable
/m20/sim/backend_ready: true
active floor: F1, generation: 1
online SCAN collision map ready
/m20/control/collision_guard_state: CLEAR
/m20/control/safety_state: NAVIGATION
```

机器狗完成站立后才放行里程计和导航。静止采样位姿为
`(-36.9979, 0.0034, 0.5642)`，相对 `(-37, 0)` 初值偏移约 4 mm；线速度和角速度
均接近零，未出现启动倒地或持续漂移。

## 3. 原生 RViz 目标闭环

直接向 `/move_base_simple/goal` 发布：

```text
start = (-37.0, 0.0)
goal  = (-35.0, 0.0)
```

结果：

- 初次规划及两次原生周期重规划均 `final_plan_success=1`；
- controller 输出经过完整 M20 安全/运动链，MuJoCo 机体实际运动；
- 规划器到达 `WAIT_TARGET`，停车制动重新接合；
- 最终位置 `(-35.1578, -0.0101)`，XY 误差约 `0.158 m`；
- 终点保护状态为 `CLEAR`；
- 未出现 `A-star error`、`The robot is inside an obstacle` 或实体碰撞错误。

一次 refined trajectory 加速度略超 `1.5 m/s^2`，集成保护将时间统一放大 1.10 倍后
通过动态可行性检查。该扩展只改变时间参数化，不改变 SCAN 生成的几何路径和障碍距离。

## 4. Typed Action 返回起点

通过自动巡检使用的正式接口发送：

```text
action: /m20/navigation/navigate
goal_id: mujoco_return_to_start
floor_id: F1
map_generation: 1
target: (-37.0, 0.0)
```

结果：

```text
status: SUCCEEDED
success: true
error_code: ERROR_NONE (0)
message: native SCAN goal reached
final_distance: 0.1771765 m
```

独立终点采样为 `(-36.8546, 0.0366, 0.5642)`，XY 误差约 `0.150 m`；姿态保持
站立，最终实测速度接近零，collision guard 为 `CLEAR`，navigation state 为
`SUCCEEDED`。

## 5. 观察项与放行结论

- 空闲或轨迹结束后，原控制器停止持续发布速度；适配层会按设计打印节流后的
  `raw/safe command timed out` 并主动输出零速度。少量调度抖动还触发过瞬时
  `ODOM_STALE`/hold，下一帧立即恢复，两个目标均未失败。这是 fail-closed 行为，后续可
  单独降低空闲日志噪声，不应通过放宽超时来掩盖。
- Ctrl-C 后 16 个进程全部 `finished cleanly`，launch 退出码为 0，无 traceback。
- 本轮放行范围是：完整 MuJoCo 一键链路、F1 空旷区原生目标、正式 typed Action 和返回
  起点闭环。
- 本轮没有重新执行 F1 四角长距离任务、障碍密集窄道、F1/F2 地图切换或完整 11 步
  巡检，因此这些仍不能标记为本次 MuJoCo 验收完成。

总体判断：当前系统的 **SCAN 算法核心和参数已经回到可自动证明的本地原包基线**；
差异集中在 M20 必需的终止/reset/hold、双向跟踪和物理执行层。相对原包，规划层已达到
“受控适配”状态，完整系统已达到“单层基本导航闭环通过、长任务与多层场景待验收”状态。
