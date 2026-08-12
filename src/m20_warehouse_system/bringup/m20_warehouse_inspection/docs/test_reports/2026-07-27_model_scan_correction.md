# 0.7.0 官方 M20 与原生 SCAN 校正验收报告

日期：2026-07-27  
环境：Ubuntu 22.04 / ROS 2 Humble / CycloneDDS / RViz 平面运动学 profile

## 静态模型

```text
official/vendor URDF SHA-256 equality       PASS
official mesh byte equality                 17/17 PASS
ROS-facing URDF parse                       PASS
root                                        base_link
connected leg chains                        4/4
links / joints                              17 / 16
added lidar or IMU links                    0
runtime official JointState names           16/16
```

## 构建与自动测试

```text
changed-package single-thread install       5/5 PASS
direct pytest                               54 PASS
CTest targets                               29/29 PASS
m20_scan_planner launch tests               2/2 PASS
duplicate C++ core in adapter               none
default native/upstream parameter contract  PASS
optional grid-route contract                PASS
RViz display/topic contract                 PASS
```

首轮并行编译曾因两份 C++ 核心同时编译触发内存峰值。删除重复构建目标后，关键包使用
单线程 CMake 安装通过。CTest 默认写只读 `~/.ros/log` 的环境问题通过设置
`ROS_LOG_DIR=/tmp/m20_ros_test_log` 解决，不涉及源码修复。

## 默认原生 SCAN 运行

完整节点图启动 12 个非 RViz 节点，包含唯一的：

```text
/m20/navigation/scan_planner
/m20/navigation/scan_controller
```

调试/显示话题存在：

```text
/m20/sensing/sensor_cloud
/m20/navigation/grid_map/occupancy
/m20/navigation/grid_map/occupancy_inflate
/m20/navigation/grid_map/sliding_map_bbox
/m20/navigation/global_list
/m20/navigation/a_star_list
/m20/navigation/optimal_list
```

使用恢复后的上游 0.05 m / 7.5 m / 0.75 m/s 规划参数发送短目标：

```text
start                               (-42.000, 0.000, 0.590)
goal                                (-40.000, 0.000, 0.590)
first_optimize_step_success         3
final_plan_success                  3
controller trajectories received   3
final position                      (-40.152, 0.000, 0.590)
final distance                      0.152 m
final FSM                           WAIT_TARGET
```

控制器和安全 supervisor 保持各自限速，因此该规划 profile 没有扩大后端速度权限。

## RViz 与感知

```text
RViz process/OpenGL                         PASS
left/right docks enabled                    PASS
SCAN occupancy QoS pub/sub                  BEST_EFFORT / BEST_EFFORT
SCAN optimal path QoS pub/sub               RELIABLE / RELIABLE
live sensing ready                          true
floor / generation                          F1 / 1
fresh cloud count                           1499
current-generation point count              3433
point X range                               -45.0 .. -20.016
point Y range                               -19.3 .. 19.3
```

同一 GUI 会话发送直接 SCAN 目标后，机器人到达 `x=-40.0069`，optimized path
端点保持 1 publisher / 1 RViz subscriber。RViz 启动瞬间曾打印两条 QoS 警告；稳定
端点查询证明配置应用后发布与订阅均为 Best Effort，属于默认订阅切换到保存配置时的
瞬态，不是持续丢图。

## 独立工作空间

```text
source-only preparation                     PASS
project packages copied                     8
pinned SCAN/model revisions                 PASS
CPU patch hash/check/apply                  PASS
resolved packages-up-to closure             18
new 18/18 fresh colcon in this executor     NOT COMPLETED
```

最后一项未标记 PASS：本会话沙箱内 `colcon` 连续停在 CMake 子进程已完成后的返回层，
包括普通安装和 symlink-install；已终止对应验证进程，没有改动项目源码。当前工作区
逐包构建、全部测试和运行时验收均通过，但 0.7.0 的归档级独立 clean build 仍应在普通
终端复跑。

本报告只覆盖 RViz 平面 profile，不代表 Gazebo 雷达、动力学或实机 M20 已验收。
