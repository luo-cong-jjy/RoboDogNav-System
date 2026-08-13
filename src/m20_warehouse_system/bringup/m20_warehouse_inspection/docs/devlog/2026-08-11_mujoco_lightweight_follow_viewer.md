# MuJoCo 轻量同步跟随镜头优化记录

日期：2026-08-11  
平台：WSL2、Ubuntu 22.04、ROS 2 Humble  
GPU：NVIDIA GeForce GTX 1660（WSLg D3D12 硬件加速）

## 1. 问题与根因

原实现使用 `mujoco.viewer.launch_passive()`，并在物理后端中调用
`viewer.sync()`。镜头使用 MuJoCo `mjCAMERA_TRACKING`，其内置平滑会使目标点
相对机体滞后；同时 `launch_passive` 自己持有一个不受项目 `max_fps` 真正约束的
后台渲染线程。在本机双层场景中，即使只降低状态同步频率，viewer 进程仍约占满一个
CPU 核。

显卡检查结果为 `Accelerated: yes`，OpenGL renderer 是
`D3D12 (NVIDIA GeForce GTX 1660)`，因此不是 Mesa 纯软件渲染。瓶颈主要是 WSLg 下
MuJoCo 通用 viewer 的后台渲染和完整仿真共同竞争调度资源。

测试期间曾出现 `EXCESSIVE_TILT`；相同的完整链路关闭窗口后也出现过同类冷启动失败，
故不能把该故障直接归因于镜头。最终验收必须同时满足后端无故障、站立稳定和导航闭环，
而不能只观察窗口是否打开。

## 2. 实施方案

### 2.1 物理与显示进程隔离

- `m20_mujoco_backend` 始终以 headless 方式运行 1 kHz 动力学；
- 新增 `m20_mujoco_viewer`，只订阅 `/m20/sim/body_pose` 和
  `/joint_states`；
- 显示进程加载只读模型副本，只调用 `mj_forward`，绝不调用 `mj_step`，也不发布
  控制或位姿；
- 显示进程以 Linux nice 10 运行，调度优先级低于物理后端和官方 ONNX 策略。

### 2.2 真正限帧的 MuJoCo OpenGL 窗口

显示层不再使用 `launch_passive`，而是使用 MuJoCo `mjv_updateScene`、`mjr_render`
和 GLFW 构成单线程窗口循环。`mujoco_viewer_max_fps` 默认 30，现在同时约束状态镜像、
场景更新和交换缓冲，不再只是限制 `sync()` 调用。

默认镜头参数：

```text
distance  = 4.0 m
azimuth   = 90 deg
elevation = -89 deg
mode      = FREE camera + direct lookat(base_link)
```

这是一种近似正上方的直接跟随方式，不使用原生 TRACKING 平滑。左键可旋转，中键可
缩放，右键可平移视角，滚轮可缩放，Esc 可关闭窗口。每帧都会把 `lookat` 重新对准
`base_link`，所以镜头不会逐渐丢失机器人。

显示副本默认关闭阴影、反射、多重采样和透明碰撞几何。这些修改不作用于物理模型，
不会改变障碍碰撞、轮地接触、规划点云或控制结果。

## 3. 性能对比

在相同的官方策略 + MuJoCo + 地图/安全链、规划器关闭的稳定空闲工况采样：

| 显示实现 | viewer CPU | viewer nice | 说明 |
| --- | ---: | ---: | --- |
| `launch_passive` 独立进程 | 90.1% | 10 | 后台渲染线程不受项目帧率真正约束 |
| 轻量 GLFW、30 FPS | 39.3% | 10 | CPU 下降约 56% |

轻量窗口样本中物理后端 `ready=true`、`fault=""`，机体位置约
`(-36.995, 0.003, 0.564)`，未发生持续漂移或倾倒。窗口开销仍不会为零；WSLg 和
桌面合成器状态仍可能影响视觉帧间隔，但不会再把窗口同步函数放进物理循环。

## 4. 完整一键链路复验

隔离域 `ROS_DOMAIN_ID=190`，正式链路开启 SCAN planner 和 30 FPS 窗口：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_mujoco.launch.py \
  use_rviz:=false \
  use_mujoco_viewer:=true \
  run_acceptance:=false
```

向 `/move_base_simple/goal` 发送 F1 空旷区目标 `(-35.0, 0.0)`：

- 三段原生 SCAN 规划均 `final_plan_success=1`；
- FSM 最终进入 `WAIT_TARGET`；
- 最终位姿 `(-35.1427, -0.0469, 0.5642)`，XY 误差约 0.150 m；
- `/m20/sim/backend_fault` 为空；
- `/m20/control/collision_guard_state` 为 `CLEAR`；
- 跟随窗口在规划、运动、重规划和停车期间持续运行；
- Ctrl-C 后新 viewer 与物理后端均 `finished cleanly`。

本轮验证的是镜头性能、单目标运动跟随及其对完整链路的隔离性，不替代多目标、窄道或
多层巡检验收。

## 5. 可调参数

主入口参数保持集中：

```text
mujoco_viewer_distance:=4.0
mujoco_viewer_azimuth:=90.0
mujoco_viewer_elevation:=-89.0
mujoco_viewer_max_fps:=30.0
```

若某台机器仍受 WSLg/远程桌面限制，可只把
`mujoco_viewer_max_fps:=20.0`；该参数不会改变物理步长、官方策略频率和导航参数。

