# 场景 1 复制与巡检切图融合验收

日期：2026-07-28

## 地图资产

- F1/F2 原始 PCD：各 144742 点。
- PCD SHA-256：
  `80604d53502ba4983889d8f43d26f4d5d4e541e9c40e5d83845f43a6d93683e8`。
- PGM SHA-256：
  `3adf0f5444f15b2cf2f72484dca975e854547dc8a655a20b67e48ba73d817b27`。
- 两层 PCD、PGM 和 intensity 逐项一致。

## 构建与自动测试

- `colcon build --symlink-install --packages-up-to m20_warehouse_inspection`：
  18 包通过。
- `m20_inspection_core` 与 `m20_warehouse_inspection` 测试通过。
- 当前工作区汇总：`228 tests, 0 errors, 0 failures, 0 skipped`。

## F1→F2 切换

执行一次事务切换回归后：

- F1 原版渲染点数：135240，边界 `(-45,-20,0)` 至 `(-5,20,2)`。
- F2 原版渲染点数：135240，边界 `(5,-20,0)` 至 `(45,20,2)`。
- 原渲染器收到 `Reloading global point cloud`。
- generation 从 1 精确递增到 2。
- 楼层保持期间无非零安全速度。
- 返回 `PHASE5_SWITCH_STRESS_PASS`。

## 完整自动巡检

使用原生 SCAN 链完成：

```text
F1_A -> F1_B -> E1:F1->F2 -> F2_A -> F2_TERMINAL
```

结果：

- 五个步骤全部完成；
- pause/resume 零速安全检查通过；
- PCD 切换、位姿迁移、SCAN reset 和新点云等待均完成；
- 最终状态 F2、generation 2；
- 最终电梯大厅误差 0.195 m；
- 返回 `PHASE4_FULL_ACCEPTANCE_PASS`。
