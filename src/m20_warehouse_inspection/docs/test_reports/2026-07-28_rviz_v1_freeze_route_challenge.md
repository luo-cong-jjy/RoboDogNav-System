# RViz 1.3.0 冻结与路线挑战静态验收报告

日期：2026-07-28

## 结果摘要

| 项目 | 结果 |
|---|---|
| RViz 1.3.0 版本与工作空间锁一致 | PASS |
| 稳定配置、地图和官方 URDF 哈希 | PASS |
| 路线挑战配置 schema | PASS |
| F1/F2 确定性资产生成 | PASS |
| 每场景 252 个障碍物（240 随机 + 12 固定） | PASS |
| 最小障碍物边界间距 ≥ 0.70 m | PASS，实测 0.702172 m |
| 7 类名义任务航段均被固定障碍物截断 | PASS |
| 0.30 m 安全轮廓膨胀后任务点连通 | PASS |
| F2 内部布局复制 F1 | PASS |
| 共享原点切图、无 teleport 和 11 步任务契约 | PASS |
| Python 单元/集成测试 | PASS，87 项 |
| ament_flake8 | PASS，49 个文件 |
| ament_pep257 | PASS |
| `m20_warehouse_inspection` 构建 | PASS |
| 安装空间地图和 launch 解析 | PASS |
| 路线挑战完整 ROS 11 步运行 | NOT RUN，本轮不冒充通过 |

## 配置与资产校验

```bash
PYTHONPATH=src/m20_warehouse_inspection \
  /usr/bin/python3 \
  src/m20_warehouse_inspection/scripts/m20_generate_maps \
  --config \
  src/m20_warehouse_inspection/config/route_challenge_system.yaml \
  --package-root src/m20_warehouse_inspection
```

结果：

```text
F1: 192314 points, 252 obstacles
F2: 192314 points, 252 obstacles
generated asset validation: PASS
```

安装空间再次验证：

```text
F1 sha256=d03b76325a569891173b2b6a96d8018604f6b2837651cfce44e8f4bbb5d42c46
F2 sha256=46c89bc2444a4acfe08f5c704fad0daf0e9668305483e498b3d33b1e8a00c216
```

## 自动测试

```bash
PYTHONPATH=src/m20_warehouse_inspection:src/m20_inspection_core:\
src/m20_scan_navigation:src/m20_multifloor_map \
  /usr/bin/python3 -m pytest -q \
  src/m20_warehouse_inspection/test \
  src/m20_inspection_core/test \
  src/m20_scan_navigation/test \
  src/m20_multifloor_map/test
```

结果：

```text
87 passed in 3.50s
```

新增断言包括：

- 冻结文件 SHA-256 与 `rviz_v1_baseline.yaml` 一致；
- 路线挑战 profile 不属于冻结集合；
- 12 个固定障碍物及 252 总数与元数据一致；
- 所有 7 类名义任务航段至少命中一个固定障碍物；
- F1/F2 分别按 0.30 m 膨胀后，门洞和任务锚点仍连通；
- 独立 launch 选择正确系统配置。

## 风格与构建

```text
ament_flake8: 49 files checked, no problems
ament_pep257: no problems
colcon build --symlink-install --packages-select m20_warehouse_inspection:
  1 package finished
```

## 验收边界

本报告只证明新场景适合进入运行测试，不证明原生 SCAN 已在该场景完成 11 步任务。
后续应先执行单次完整节点图验收，再决定是否执行 10 次任务和 20 次切换压力回归。
