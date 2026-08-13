# 2026-08-10 源码闭包收敛与工作空间清理

## 架构结论

项目继续以 `m20_warehouse_inspection` 作为唯一用户功能入口，不把所有源码机械合成一个
ROS package。`rosidl` 接口、官方模型资源、C++ SCAN 库、Python 状态机和带独立许可证
的第三方包需要保持边界，否则会形成循环依赖、重复源码和无法单独替换的实机接口。

默认 colcon 发现范围已从 39 个 ROS 包收敛为 `workspace_lock.yaml` 中锁定的 22 包：
10 个项目模块和 12 个 SCAN/SDK 依赖。八个顶层旧版/实验包放置 `COLCON_IGNORE`；
隔离工作空间准备脚本也会屏蔽上游仓库中的 Go2、Lite3 和原版 SCAN demo 包。

## 已清理内容

- 删除旧工作空间 `build/`、`install/`、colcon `log/` 后执行全新构建。
- 删除根目录和源码树中的 pytest/Python cache。
- 删除 `src/third_party` 内嵌套遗留的 colcon/CMake build、install、log。
- 删除 3D-Nav、GTSAM、OSQP、Sophus、Livox-SDK2 中的可再生构建目录。
- 删除空目录 `src/deep_robotics_file` 和旧 `symlink_install_manifest.txt`。
- 第三方源码目录由约 3.9 GiB 降为约 2.6 GiB。

以下内容明确保留：

- `docs/devlog` 和 `docs/test_reports` 的过程记录与结论。
- `artifacts` 中的导航/间距测试 CSV、summary 和部署归档。
- MuJoCo RL checkpoint、metrics 与训练配置；它们虽位于 `logs` 命名目录，但属于模型
  资产，不能按普通运行日志删除。
- 实机雷达 PCD/dataset 和官方技术资料。

## 验证

- 全新构建：22/22 包成功。
- 项目测试：503 tests，0 errors，0 failures，1 skipped。
- 无界面自由导航：2.5 m 目标 6.98 s 成功，最终误差 0.150 m，3 条 B-spline，
  collision/backend fault 均为 0。
- 测试结束后无残留 ROS 进程，原始临时运行日志已删除。

