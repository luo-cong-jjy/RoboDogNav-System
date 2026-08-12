# F2 双点巡检覆盖与返回起点开发记录

日期：2026-07-28  
版本：1.2.0  
范围：任务配置、任务类型、验收与回归契约

## 1. 目标

第二场景不能只到达 F2_A。由于 F2 的内部地图是 F1 的复制，默认巡检覆盖应同样包含
局部坐标一致的 A、B 两个巡检点，并在结束后返回进入 F2 时的共享原点 `(0,0)`。

同时保持以下约束：

- 不修改 SCAN 的目标、重规划、B 样条或闭环控制链；
- 不增加、删除或重新分布障碍；
- 不使用仿真瞬移；
- 返回路线应使用已经由地图资产验证的净空走廊。

## 2. 七步任务

新的默认 sequence：

```text
1  F1_A                         inspection + dwell
2  F1_B                         inspection + dwell
3  E1:F1->F2                    navigate origin + in-place map switch
4  F2_A                         inspection + dwell
5  F2_B                         inspection + dwell
6  F2_RETURN_VIA_A              transit, no dwell
7  F2_TERMINAL                  shared origin, no dwell
```

F2_A 和 F2_B 与 F1_A、F1_B 使用相同的 floor-local 坐标，因此两层巡检覆盖对称。

## 3. 为什么增加 transit

若从 F2_B 直接回原点，会形成一条新的长对角线，可能要求地图生成器重新排布与该线
冲突的障碍。为保持用户已经看到的障碍密度和布局不变，回程先经过 F2_A，再沿已有
`F2_A <-> origin` 走廊返回。

新增 `transit` 任务类型具有以下语义：

- 引用已有 inspection point 的位姿；
- 使用相同 typed `NavigateFloor` 和 native SCAN 链；
- 不执行巡检 dwell；
- 在 MissionState/RViz 中使用独立名称显示；
- 失败、暂停、停止和 retry 规则与普通导航步骤一致。

## 4. 动态步骤数

phase-4 acceptance 和 phase-5 mission stress 不再写死 `5`：

- phase-4 从最终 typed MissionState 的 `step_count` 获取期望值；
- phase-5 从系统配置的 mission sequence 长度获取期望值。

这样以后增加巡检点时不需要再次修改验收常量。

## 5. 障碍资产不变证明

加入 F2_B 和回程 transit 后重新执行确定性地图生成。四个核心资产哈希保持不变：

```text
F1 PCD  1763a3ceefba9e7a480dc53077f822532e51aed841b707d4f7064b4f89b83054
F2 PCD  34ea9a1158dbc13a37a6150892a51d48f05da92aa71ccdfd0e980411e3fe7f8b
F1 PGM  7e12ff6e847c071ead9716789ff24e1533f1a73a7bdf224b6da6cee7b00dda2a
F2 PGM  60dc019651919dae883f1aee0e6de31ca3925ccb179aa4683d81478eeb82ce22
```

两层仍各有 180 个障碍、144586 个点。本次修改没有改变障碍总密度或局部分布。

## 6. 修改文件

```text
config/flat_multifloor_system.yaml
m20_inspection_core/mission_policy.py
m20_inspection_core/mission_executor_node.py
m20_warehouse_inspection/configuration.py
m20_warehouse_inspection/map_assets.py
m20_warehouse_inspection/phase4_acceptance_node.py
m20_warehouse_inspection/phase5_regression_node.py
相关 unit/contract tests
README.md、interface_contract.md、system_plan.md、CHANGELOG.md
```

验收状态见
`docs/test_reports/2026-07-28_f2_full_coverage_return.md`。

