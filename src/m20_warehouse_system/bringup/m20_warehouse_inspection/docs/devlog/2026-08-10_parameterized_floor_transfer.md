# 2026-08-10 多层切换参数化与运输适配器封装

## 目标

将“楼层地图原子切换”与“机器人如何从一层到另一层”分离。默认双平面区域方案必须保持
现有效果；后续改用真实电梯、升降平台或其他方案时，不修改巡检任务、地图 generation
事务、导航 reset 和安全保持逻辑。

## 当前默认流程

默认 `dense_four_corner_system.yaml` 与 `flat_multifloor_system.yaml` 使用：

```yaml
floor_switch:
  transfer_adapter: timed_hold
  pose_handoff: preserve
  transition_delay_sec: 0.0
  transfer_timeout_sec: 120.0
  external_action_name: /m20/floor_transfer/execute
```

其运行顺序为：

```text
任务导航到 route.source_trigger_pose
  -> 校验 active floor / generation / trigger pose
  -> floor_switch_hold=true 并确认机器人连续停稳
  -> reset 旧楼层 SCAN 状态
  -> timed_hold（默认 0 秒，不产生瞬移）
  -> generation compare-and-swap 提交目标 active PCD/occupancy
  -> preserve 校验 odom 仍位于 target_release_pose
  -> reset 目标楼层 SCAN 状态
  -> 等待目标 floor/generation 的新鲜局部点云
  -> 成功后 floor_switch_hold=false
```

两个平面区域虽然在 RViz 中同时显示，但规划与局部感知只使用 active floor。默认 F1 和
F2 的 route trigger/release 都是共享原点 `(0, 0, 0)`，因此切换只改变活动地图，不传送
机器狗。任务步骤使用通用名称：

```yaml
- type: floor_transfer
  connector: E1
  from: F1
  to: F2
```

旧的 `elevator_transfer/elevator` 和 `SwitchFloor.goal.elevator_id` 为已有 ROS 接口兼容
名称，运行时均按通用 connector id 解释。配置中的 `elevators` 也是兼容保留的 connector
注册表名称，不代表 transport provider 必须是电梯。

## 模块边界

| 模块 | 职责 | 禁止承担的职责 |
| --- | --- | --- |
| mission executor | 到达 approach/trigger，调用 `SwitchFloor` | 直接切 PCD、传送位姿、操作电梯 SDK |
| floor switch manager | hold、停稳确认、transport 调度、地图 CAS、pose handoff、导航/感知恢复 | 站点硬件协议、巡检路线规划 |
| map manager | 按 expected generation 原子提交目标地图 | 机器人运输、导航控制 |
| transport provider | 执行真实电梯/升降平台/站点流程并报告完成 | 切图、释放 hold、修改任务游标 |
| pose handoff | 确认或建立目标楼层定位初值 | 物理运输、地图 generation 提交 |

运输 provider 使用内部 Action `ExecuteFloorTransfer`。输入包含 connector、起止楼层和源
generation；输出是成功、错误码和文本，feedback 可发布 provider 自己的阶段。协调器
负责超时、取消和失败后的停车保持。

## 配置层级

配置按“系统默认 < connector 覆盖 < 有向 route 覆盖”合并。未写覆盖项时继承上一级。

```yaml
elevators:
  LIFT_A:
    trigger_tolerance_xy: 0.35
    trigger_tolerance_yaw: 0.50
    transfer:
      adapter: external_action
      pose_handoff: wait_for_target
      timeout_sec: 90.0
      external_action_name: /site/lift_a/execute
    transitions:
      - from: F1
        to: F2
        source_trigger_pose: [1.2, 3.0, 1.57]
        target_release_pose: [0.8, -2.0, 1.57]
      - from: F2
        to: F3
        source_trigger_pose: [0.8, -2.0, 1.57]
        target_release_pose: [4.0, 1.0, 0.0]
        transfer:
          timeout_sec: 150.0
```

`transitions` 是显式有向边，因此可描述两层、多层、单向或多个 connector；不再依赖固定
F1/F2 正反推导。重复有向边、任务引用不存在的边、未知策略和非法超时会在启动配置校验
阶段拒绝。

## 可选策略

transport adapter：

- `timed_hold`：只在安全 hold 中等待配置时长，适合当前共享点仿真或外部人工完成的测试。
- `external_action`：调用 `ExecuteFloorTransfer` provider，适合真实电梯、升降平台和站点
  PLC/SDK。

pose handoff：

- `preserve`：不改变 odom，并验证当前位姿等于目标 release；当前默认。
- `set_simulation_pose`：调用 `/m20/sim/set_pose`，只用于分离区域仿真。
- `wait_for_target`：等待定位后端报告到达目标 release，适合真实运输后重定位。
- `none`：provider/定位系统拥有位姿交接，协调器不重复验证；仅在外部接口有同等安全保证
  时使用。

## 故障与恢复

transport、地图提交、位姿交接、导航 reset 或新鲜点云等待任一步失败，协调器均保留
`floor_switch_hold=true`。若目标地图已经提交但后续阶段失败，retry 不会再次运输或增加
generation，而是进入 `RECOVERING_COMMITTED_TARGET`，重做 pose handoff、目标导航
reset 和感知恢复。只有完整事务成功才释放 hold。

## 兼容与移植

旧 profile 未增加 `floor_switch` 或显式 `transitions` 时，解析器继续从
`simulation.teleport_on_floor_switch`、`elevator_transition_delay_sec` 和旧的
source/target/reverse 字段生成相同策略。这样测试场景无需一次性迁移；新功能应使用
`floor_transfer`、显式有向 route 和独立 provider。

