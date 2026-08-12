# 2026-08-12 实机 drdds、安全与建图开发记录

## 目标

对齐最近背部主机 drdds，补齐两种厂家运动后端的硬急停/设备故障保护，并把 F1 从建图到
SCAN 静态 PCD 的实际部署步骤变成可执行流程。

## 修改摘要

- 活动 drdds 对齐 v1.2.0 背部基准：`stamp`、扁平 MotionInfo、移除 Timestamp；
- 更新 direct ROS、MuJoCo、官方 SDK 源码和锁定 SDK 补丁；
- basic_server 解析 HES 与异步 ErrorList；两后端增加释放后人工确认闭锁；
- 新增单层 surveyed 硬件配置模式和真实 PCD 导入/校验；
- 新增 Elevator-LIO relocation 根配置及稳定 F1 PCD 名；
- 重写部署手册中的建图、保存、重定位、地图导入、急停验证和故障诊断命令。

## 验证记录

- 全部 25 个 drdds `.msg` 规范化 ABI 与背部基准一致；
- `validate_foxy_hardware_source.py` 对 basic_server/direct_ros 均 PASS；
- 固定 SDK 补丁已在官方 revision `ee289d4...` 的干净临时克隆上通过
  `git apply --check`，并更新 SHA-256；
- Python 模块通过 `py_compile`；
- 单层硬件模板通过 schema 校验；其缺少真实 PCD 时 `--require-assets` 按设计失败；
- `/tmp` 干净 build/install 中 4 个受影响包构建成功；colcon/ament 受影响包完整回归
  `327 tests, 0 errors, 0 failures, 1 skipped`，最后一轮硬件契约复验
  `324 tests, 0 errors, 0 failures, 1 skipped`；
- 隔离发布脚本从固定三方 revision 成功重建源码并应用锁定补丁；
- 安装后的 `m20_import_site_pcd` 成功导入 185392 点回归资产；
- 当前 shell 没有独立 `pytest` 命令，因此测试统一通过 colcon/ament 执行。

## 未宣称完成

未在 M20-PRO 目标机验证 Foxy 编译、Fast DDS QoS、HES 现场状态、basic_server 报文、
Elevator-LIO 真实建图、物理急停或实体运动。所有实机动作必须按 04 手册门控进行。
