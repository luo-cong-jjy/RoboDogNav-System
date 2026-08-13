# Launch 文件

## 1. 推荐入口

1. `mujoco_sdk_bringup.launch.py`

   启动 MuJoCo 仿真、官方 `m20_sdk_deploy/rl_deploy_cmdvel`、安全仲裁、运动适配和 RViz。

2. `control_core.launch.py`

   只启动 `cmd_vel_safety_mux_node` 和 `motion_adapter_node`。

3. `factory_sim.launch.py`

   只启动本项目 MuJoCo 仿真节点。

## 2. 单模块调试入口

1. `cmd_vel_safety_mux.launch.py`：只启动速度安全仲裁。
2. `motion_adapter.launch.py`：只启动运动适配层。

## 3. 兼容入口

1. `mujoco_sdk_inspection.launch.py`

   旧名字保留，但现在只转到 `mujoco_sdk_bringup.launch.py`。
   MuJoCo 包不再启动巡检任务、waypoint 跟踪或 Nav2。

2. `inspection_core.launch.py`

   旧名字保留，但现在只转到 `control_core.launch.py`。
