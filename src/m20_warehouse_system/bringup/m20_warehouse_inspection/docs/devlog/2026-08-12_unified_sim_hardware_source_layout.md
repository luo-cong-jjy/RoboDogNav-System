# 2026-08-12 仿真/实机统一源码目录整理

## 目标

将原本散落在工作区 `src/` 顶层的项目自有 ROS 包收敛为一个可交付目录，
同时保持仿真和实机共用一套规划、导航、运动适配、安全和任务代码。

## 实施内容

- 新建 `src/m20_warehouse_system/`。
- 按 `common/navigation/motion/safety_mission/simulation/bringup` 分类原有项目包。
- 保留所有 ROS 包名、节点名、话题名和 launch 文件名。
- `SCAN-Planner`、云深处 SDK、`drdds` 和 `Elevator-LIO` 保持为外部/厂家边界。
- 隔离工作区生成器现在整体复制 `m20_warehouse_system`，不再逐个从 `src/` 顶层拼装项目包。
- 独立 RL 训练工程纳入 `simulation/` 开发分类，但不进入运行部署工作区。
- Foxy 源码预检改为递归发现 ROS 包，并使用新的项目目录映射。

## 运行边界

仿真与实机的上层节点图不分叉，只替换：

1. 位姿和局部点云来源；
2. 安全速度之后的最终执行后端。

RViz 使用 `m20_warehouse_sim`，MuJoCo 使用 SDK/ONNX +
`m20_mujoco_backend`，实机使用 `Elevator-LIO` + `basic_server/direct_ros`。

## 验证

- `colcon list --base-paths src` 在移动后仍发现原来的全部活动包，无重名活动包。
- 9 组项目 Python 契约/策略测试共 241 项通过，其中包含
  `m20_scan_planner` 的 2 项 ROS launch 启动测试。
- Foxy `basic_server` 和 `direct_ros` 源码预检都通过。
- 从空的 `/tmp` build/install 目录构建生产闭包：22 个包全部通过。
- 刷新工作区日常 `build/install`：22 个包全部通过，并移除整理前路径留下的
  81 条失效 install 软链接；当前失效软链接数为 0。
- 从该全新 install 启动无界面 MuJoCo 一键链路：14 个进程启动，SDK
  stand-up 完成，`/m20/sim/backend_ready=true`，F1 地图状态为 `READY`，随后
  Ctrl-C 正常关闭全部进程。

完整证据见
[`../test_reports/2026-08-12_unified_source_layout_regression.md`](../test_reports/2026-08-12_unified_source_layout_regression.md)。
