# 2026-08-12 仿真/实机统一源码目录回归

## 验证对象

- 项目自有源码收敛到 `src/m20_warehouse_system/` 后的 ROS 包发现与依赖关系；
- 仿真、实机共用 Python 策略和 launch 契约；
- Humble 生产闭包的全新构建；
- 完整 MuJoCo + 官方 SDK/ONNX 一键启动链路。

## 静态与单元测试

- 9 组测试合计 `241 passed`；
- `m20_scan_planner` ROS launch 测试 `2 passed`；
- Foxy `basic_server` 源码预检：PASS；
- Foxy `direct_ros` 源码预检：PASS；
- `prepare_isolated_workspace.sh` shell 语法检查：PASS。

测试使用 `ROS_LOG_DIR=/tmp/m20_ros_log`，避免验证环境只读的
`~/.ros/log`。这只改变日志落盘位置，不改变 ROS 节点行为。

## 全新构建

使用空的 `/tmp/m20_layout_build` 和 `/tmp/m20_layout_install`，执行：

```bash
colcon --log-base /tmp/m20_layout_colcon_log build \
  --base-paths src \
  --build-base /tmp/m20_layout_build \
  --install-base /tmp/m20_layout_install \
  --symlink-install \
  --packages-up-to m20_warehouse_inspection
```

结果：`Summary: 22 packages finished`。6 个第三方 SCAN 包保留原有编译告警，
没有构建失败或新目录引用错误。

随后使用相同 22 包闭包刷新工作区原有 `build/install`，结果同样全部通过。整理前
遗留的 81 条失效 symlink（旧 launch、旧 PGM/YAML 和旧调试脚本）已从 install
产物中清理；`find install -xtype l` 最终为 0，主 launch 的 symlink 指向新的
`src/m20_warehouse_system/bringup/m20_warehouse_inspection`。

## MuJoCo 一键链路

从 `/tmp/m20_layout_install/setup.bash` 启动：

```bash
ros2 launch m20_warehouse_inspection inspection_mission_mujoco.launch.py \
  use_rviz:=false use_mujoco_viewer:=false
```

结果：

- 地图、M20 description、local sensing、SCAN planner/controller、导航网关、
  安全监督、任务/切层、运动适配、官方 SDK 和 MuJoCo 后端共 14 个进程启动；
- SDK motor command 被 MuJoCo 接收，stand-up 稳定后后端解除门控；
- `/m20/sim/backend_ready` 为 `true`；
- `/m20/sim/body_pose` 有连续有效位姿，抽样位置约
  `(-36.998, 0.003, 0.564)`；
- `/m20/map/state` 为 `floor_id=F1, generation=1, ready=true, phase=READY`；
- 无目标点时 SCAN FSM 正常保持 `INIT / wait for goal`；
- Ctrl-C 后所有 14 个进程均 clean exit。

首次在受限沙箱内运行时 CycloneDDS 无权枚举网络接口；切换到正常 DDS 网络权限
后即通过，确认它不是源码目录调整造成的故障。
