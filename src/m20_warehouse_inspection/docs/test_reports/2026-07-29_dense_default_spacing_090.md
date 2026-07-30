# 正式双场景0.90 m障碍物间距回归报告

日期：2026-07-29

## 结果

| 检查项 | 结果 |
|---|---|
| 配置完整性与资产一致性 | PASS |
| F1/F2障碍物数量均为246 | PASS |
| F1/F2实际最小本体边界距离≥0.90 m | PASS，均为0.900236 m |
| F2内部布局复制F1 | PASS |
| 0.30 m栅格膨胀后任务锚点连通 | PASS |
| 冻结配置、PCD和PGM哈希 | PASS |
| ROS包重建 | PASS |
| 包级回归 | PASS，149项，0失败 |

验证命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

PYTHONPATH=src/m20_warehouse_inspection \
python3 src/m20_warehouse_inspection/scripts/m20_validate_config \
  --config \
  src/m20_warehouse_inspection/config/dense_four_corner_system.yaml \
  --package-root src/m20_warehouse_inspection \
  --require-assets

colcon build --symlink-install \
  --packages-select m20_warehouse_inspection
colcon test --packages-select m20_warehouse_inspection
colcon test-result \
  --test-result-base build/m20_warehouse_inspection --verbose
```

## 资产统计

- F1：185392个点，246个障碍物，最近边界距离0.900236 m；
- F2：185392个点，246个障碍物，最近边界距离0.900236 m；
- 两层配置阈值均为0.90 m；
- 障碍物数量未因提高间距而降低。

## 尚未覆盖

本报告不包含新0.90 m资产上的完整MuJoCo 11步动态任务。历史0.70 m场景的
11/11结果继续作为历史记录保留，不计作本次动态通过。下一次系统运行应重点记录：

- 11步是否无重试完成；
- 每步耗时和最终返航误差；
- `collision_guard`触发次数；
- 局部规划停滞或超时次数；
- MuJoCo接触和SDK后端故障次数。
