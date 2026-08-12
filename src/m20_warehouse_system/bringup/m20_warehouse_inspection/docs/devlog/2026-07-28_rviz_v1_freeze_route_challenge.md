# RViz 1.3.0 基线冻结与路线受扰动场景开发记录

日期：2026-07-28  
稳定版本：1.3.0  
实验 profile：`route_challenge`

## 1. 目标

本轮同时处理两项工作：

1. 把总技术路线同步到当前双场景四角 11 步任务，并冻结已验收的 RViz 稳定基线。
2. 不修改冻结地图，新增一套静态障碍物进入名义巡检线的测试场景，为后续验证
   SCAN 绕障和多轮任务稳定性提供独立入口。

Gazebo、实机 SDK 和路线挑战的 ROS 多轮运行不在本轮验收范围。

## 2. 原场景为何显得刻意

地图生成器会为任务锚点和 native-SCAN 名义直达航段建立保护区域，随机障碍物不会
进入保护区域。`dense_four_corner` 的 6 个固定障碍物虽然靠近下、右、上三段过道，
但仍给中心线保留约 0.8 m 净空。因此它适合作为稳定任务基线，却不适合观察规划器
主动偏离直线路线。

本轮没有取消随机地图的安全保护，而是在独立配置中使用明确命名的固定障碍物进入
名义航段。这样可重复、可审计，也不会因更换随机种子意外堵死巡检点或门洞。

## 3. 路线挑战场景

配置：

```text
config/route_challenge_system.yaml
launch/route_challenge_mission_rviz.launch.py
maps/route_challenge/
```

它复用高密度场景的双区域尺寸、相邻原点、四角点、11 步任务、SCAN 参数和安全参数。
F2 仍为 F1 内部布局副本，两个场景只有相邻边门洞方向不同。

每场景包含：

```text
240 个确定性随机障碍物
12 个显式路线障碍物
252 个障碍物合计
```

12 个固定障碍物尺寸为 0.8～1.3 m 的矩形，方向和位置不完全一致，覆盖：

| 名义航段 | 固定障碍物 |
|---|---|
| F1 初始点到左下角 | `left_descent_case` |
| F2 门洞到左下角 | `inbound_left_pallet` |
| 下边巡检通道 | `bottom_west_crate`、`bottom_east_rack` |
| 右边巡检通道 | `right_lower_pallet`、`right_upper_case` |
| 上边巡检通道 | `top_east_crate`、`top_west_rack` |
| 左上角到共享门洞斜线 | `outbound_diagonal_west`、`outbound_diagonal_east` |
| 反向切回后的最终返航线 | `home_aisle_east`、`home_aisle_west` |

固定障碍物故意与名义中心线相交，但巡检点、初始点和 4 m 共享门洞保持 free space。
随机障碍物和固定障碍物统一执行 0.70 m 最小边界间距检查。

## 4. 生成资产

| 场景 | 点数 | 障碍物 | 固定障碍物 | 实测最小间距 |
|---|---:|---:|---:|---:|
| F1 | 192314 | 252 | 12 | 0.702172 m |
| F2 | 192314 | 252 | 12 | 0.702172 m |

PCD SHA-256：

```text
F1 d03b76325a569891173b2b6a96d8018604f6b2837651cfce44e8f4bbb5d42c46
F2 46c89bc2444a4acfe08f5c704fad0daf0e9668305483e498b3d33b1e8a00c216
```

自动测试逐段采样 7 类名义航段，确认每一类至少与一个固定障碍物相交。随后将 PGM
按 `0.30 m` 安全轮廓膨胀，分别检查 F1/F2 的门洞、初始点和四角巡检点；所有目标
仍位于 free cell 且属于同一连通区域。

## 5. RViz 1.3.0 冻结

新增：

```text
config/rviz_v1_baseline.yaml
```

冻结清单锁定：

- `flat_multifloor_system.yaml` 和对应 PCD/PGM；
- `dense_four_corner_system.yaml` 和对应 PCD/PGM；
- 官方 M20 URDF；
- 项目版本 1.3.0；
- `NavigateFloor`、`RunMission`、`SwitchFloor`、generation 和安全速度链契约。

自动测试重新计算所有 SHA-256，任何对稳定配置、地图或官方模型的无意修改都会失败。
`route_challenge_system.yaml` 明确列入 `excluded_from_baseline`，因此实验地图可以
独立迭代而不改变 1.3.0 稳定结论。

## 6. 文档与版本

- `docs/system_plan.md` 更新为 1.7，当前主任务改为 11 步；
- `package.xml` 和 `workspace_lock.yaml` 同步为 1.3.0；
- README 增加稳定/实验 profile 边界及两步运行入口；
- CHANGELOG 将当前稳定能力归档到 1.3.0，路线挑战保留在 Unreleased。

## 7. 验证

```text
Python 单元/集成测试       87 passed
ament_flake8              49 files, no problems
ament_pep257              no problems
集成包构建                 PASS
安装空间挑战地图校验       PASS
挑战 launch 参数解析       PASS
```

详细命令和结果见同名测试报告。

## 8. 已知边界与下一步

- 本轮证明了配置、生成资产、障碍间距、名义路线相交和安全轮廓下的静态连通性。
- 尚未声称 SCAN 已完成该实验场景的完整 11 步任务；需要实际运行后记录绕行轨迹、
  短暂安全停车、失败点和完成时间。
- 后续压力测试应以独立 mission ID
  `route_challenge_four_corner_patrol` 运行，不能复用冻结高密度配置的验收结论。
- Gazebo 6A 只消费冻结接口和资产，不修改 `rviz_v1_baseline.yaml` 中的稳定文件。
