# F2 双点覆盖与返回起点验收

日期：2026-07-28  
当前状态：静态与模块验收 PASS；端到端运行待环境授权

## 1. 已完成

```text
配置、任务、地图、bringup 与回归测试：57 passed
ament_flake8：PASS
ament_pep257：PASS
地图生成与资产校验：PASS
```

契约检查：

- 默认任务包含 F2_A 与 F2_B；
- F2_RETURN_VIA_A 类型为 transit，位姿等于 F2_A，dwell 为 0；
- F2_TERMINAL 仍为共享原点；
- phase-4/phase-5 根据配置读取 7 步，不再写死 5；
- F2 的 `origin -> A -> B -> A -> origin` 四段均具有资产净空测试；
- PCD/PGM 哈希与修改前一致，障碍布局未变化。

## 2. 端到端运行

计划命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_rviz.launch.py \
  use_rviz:=false run_acceptance:=true acceptance_mode:=full
```

本次启动请求被执行环境的自动权限审批服务拒绝，返回原因是审批服务连接中断，而不是
ROS launch、节点或任务失败。根据执行安全要求未绕过审批重试。

待获得新的明确运行授权后，需要确认以下状态：

```text
F2_A DWELLING -> complete
F2_B DWELLING -> complete
F2_RETURN_VIA_A transit -> complete
F2_TERMINAL at origin -> complete
completed_steps = 7
switch_jump <= 0.02 m
final_error <= 0.25 m
```

