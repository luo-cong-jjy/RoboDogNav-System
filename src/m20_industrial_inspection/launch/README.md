# Launch 文件

当前启动文件：

- `factory_sim.launch.py`：启动本项目自定义工厂 MuJoCo 仿真。
- `mujoco_sdk_inspection.launch.py`：默认启动 MuJoCo + 官方 `m20_sdk_deploy` 控制器 + 巡检安全层；`start_mission:=true` 时只启动任务层，避免二次启动仿真和 RViz。
- `motion_adapter.launch.py`：只启动运动适配层。
- `cmd_vel_safety_mux.launch.py`：只启动速度安全仲裁。
- `inspection_core.launch.py`：启动速度安全仲裁和运动适配层。
- `patrol_demo.launch.py`：启动巡检动作演示、安全仲裁和运动适配层。
- `odom_patrol_demo.launch.py`：启动 odom 全局坐标巡检、安全仲裁和运动适配层。
- `inspection_mission_demo.launch.py`：启动巡检任务管理器、odom 目标跟踪器、安全仲裁、运动适配层和 RViz。
