# 相邻双场景共享原点切图验收

日期：2026-07-28  
结果：PASS

## 1. 静态与模块测试

```text
地图重新生成：PASS
生成资产一致性校验：PASS
核心与集成 pytest：56 passed
```

覆盖项：

- connected layout 的 F1/F2 范围与共享原点约束；
- 两侧 PCD/PGM 实体门洞和内部副本契约；
- 启动发布 active、inactive、all-floors 点云；
- SCAN 原版显示话题与默认 native 链；
- mission 直达航段障碍净空；
- 无传送 floor-switch 分支；
- 共享边界 0.35 m 容差、封闭边界仍 fail-closed。

最终地图资产：

| 项目 | F1 | F2 |
|---|---:|---:|
| 点数 | 144586 | 144586 |
| 障碍数 | 180 | 180 |
| PCD SHA-256 | `1763a3ceefba9e7a480dc53077f822532e51aed841b707d4f7064b4f89b83054` | `34ea9a1158dbc13a37a6150892a51d48f05da92aa71ccdfd0e980411e3fe7f8b` |

两份完整哈希不同是预期结果：F1 门在 `max_x`，F2 门在 `min_x`；内部 PCD、PGM 和
180 个障碍布局已由资产校验器断言完全相同。

## 2. 端到端运行

命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_rviz.launch.py \
  use_rviz:=false run_acceptance:=true acceptance_mode:=full
```

运行节点仍使用默认 `scan_native`，没有启用 `use_grid_route`。

关键状态：

```text
preloaded 2 flat floor maps
active_floor=F1, generation=1, all_points=289172

F1_A: completed
F1_B: completed
E1:F1->F2: robot navigated to shared origin
active map committed: floor=F2, generation=2
F2_A: completed
F2_TERMINAL: completed at shared origin
```

最终验收标记：

```text
PHASE4_FULL_ACCEPTANCE_PASS:
5 steps,
pause/resume,
zero-jump origin map switch,
F2 origin terminal,
generation 2,
switch_jump=0.000m,
final_error=0.142m
```

## 3. 无瞬移证据

- floor-switch manager 启动日志为 `transfer_mode=in-place shared gateway`。
- 配置 `teleport_on_floor_switch=false`，管理器不创建 `SetSimulationPose` 客户端。
- F1 到点后才出现 `ELEVATOR_TRANSFER` 和 `active map committed`。
- F2 第一条 SCAN 轨迹从 `(-0.0522, 0.0350)` 开始，仍位于原点停车误差内。
- 验收节点直接比较切换前后 odom，`switch_jump=0.000m`。

因此本次通过不是“切图后把机器人放到 F2”，而是机器人先运动到共享原点，保持同一
世界位姿完成 active map 切换。

## 4. 显示与围栏结论

- 地图服务器首轮锁存消息包含两场景，RViz 默认总览视角覆盖 `x=[-40,40]`。
- active 与 inactive 点云互斥绘制，不再对同一共面 PCD 重复着色。
- RViz 边框 Marker 在门洞处断开；真实 PCD 墙和 occupancy 边也同步开门。
- 除 `y=[-2,2]` 的相邻门洞外，其余围栏继续由 SCAN 与 collision guard 当作障碍。

