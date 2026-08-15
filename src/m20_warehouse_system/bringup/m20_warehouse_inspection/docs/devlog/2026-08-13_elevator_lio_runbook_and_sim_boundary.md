# 2026-08-13 Elevator-LIO 操作手册与仿真边界整理

## 目的

- 将 M20-PRO 建图、PCD 保存、relocation 配置与冷启动验收放到 Elevator-LIO 包内；
- 明确仿真没有在线建图，也没有一个单包一对一替代 Elevator-LIO；
- 区分“实机运行不需要仿真节点”和“当前统一源码发布闭包仍携带仿真包”。

## 文档修改

1. 新增并按现场命名整理为 `src/Elevator-LIO/Virdy-m20-pro-建图定位启动.md`；
2. 从 `src/Elevator-LIO/README.md` 增加包内入口；
3. 总部署手册增加仿真/实机职责矩阵，并链接包内权威操作说明；
4. 包清单补充 hardware launch 的保留/替换边界。

## 结论

- 仿真地图由 `m20_generate_maps` 离线生成，仿真不验证真实 LiDAR-IMU mapping；
- 位姿由 `m20_mujoco_backend` 或 `m20_warehouse_sim` 提供；
- 虚拟实时点云由 SCAN `local_sensing_node` 根据静态 PCD 和真值位姿生成；
- 实机由 Elevator-LIO mapping/relocation 替代上述定位与感知捷径；
- `m20_flat_map_server`、SCAN 核心规划、安全层和运动适配层在实机继续保留；
- MuJoCo、RViz 运动学、虚拟感知和 ONNX 仿真低层不参与 hardware launch。

当前 `prepare_isolated_workspace.sh` 仍构建统一 source-only 闭包，因此这些仿真源码会进入
release 但不运行。未在本轮实现、也未宣称已经验证 hardware-only 精简闭包，避免现场
手工删包后造成 package manifest、rosdep 或 Foxy 构建问题。

## 检查

- `git diff --check`：通过；
- 核对 `PACKAGE_ROOT_DIR`：PCD 保存和加载均指向编译时 Elevator-LIO 源码目录；
- 核对根配置：mapping 使用 `root_config_m20.yaml`，relocation 使用
  `root_config_m20_navigation_relocation.yaml`；
- 本轮仅修改文档，没有修改运行节点、launch 或参数，不需要重复启动 MuJoCo 回归。
