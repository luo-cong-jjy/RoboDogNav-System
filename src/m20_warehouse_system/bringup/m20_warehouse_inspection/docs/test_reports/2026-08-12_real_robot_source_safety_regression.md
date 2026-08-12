# 2026-08-12 实机源码与安全边界回归

## 范围

`drdds`、`m20_locomotion_control`、`m20_mujoco_backend`、
`m20_warehouse_inspection`；不启动实体机器人，不编译 Elevator-LIO。

## 环境

- Ubuntu 22.04 / ROS 2 Humble；
- `/tmp/m20_real_robot_regression_*` 全新 build/install/log；
- 工作空间已有 build 不清理、不复用。

## 结果

- 4 个包构建成功；
- colcon/ament 专项测试：受影响包完整回归曾达到 327 tests，0 errors，0 failures，
  1 skipped；最后一轮硬件契约复验为 324 tests，0 errors，0 failures，1 skipped；
- basic_server/direct_ros Foxy 源码预检均 PASS；
- 活动 drdds 和背部主机 v1.2.0 基准的 25 个消息 ABI 一致；
- SDK 锁定补丁 SHA-256：
  `41a198698c8bae00d14b9bf0e9f18cd31b912780bde8e5a1f928058fd7aa5356`；
- 隔离工作空间脚本从固定 SCAN/model/SDK revision 成功准备；
- 隔离 SDK 中硬件与 MuJoCo 时间字段均为 `header.stamp`；
- PCD CLI 回归导入 185392 点并生成 surveyed JSON/SHA。

## 边界

这份报告证明源码、构建和离线契约没有已知回归；不证明 Foxy 目标机编译、Fast DDS QoS、
HES 电气行为、basic_server 实机状态、Elevator-LIO 现场建图或实体运动安全。
