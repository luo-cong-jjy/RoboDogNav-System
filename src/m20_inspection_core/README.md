# M20 巡检安全与楼层事务核心

速度安全 supervisor 的数据流：

```text
SCAN cmd_vel_raw ----\
manual cmd_vel ------- safety supervisor -> cmd_vel_safe -> backend
e-stop --------------/
floor switch hold ---/
map ready -----------/
odom freshness ------/
collision stop ------/
```

急停、切图保持、地图未就绪和定位超时直接输出零速度，不经过普通加速度限制。命令超时
和普通停止采用配置的受控减速。

`m20_collision_guard` 对 active occupancy 按 M20 半径 0.38 m 加 0.12 m 余量膨胀，
并以前视 1.0 s 检查 SCAN 原始速度。地图或里程计未就绪时默认断言停车。该保护是独立
于 SCAN 内部 GridMap 的最后一道静态地图导航保护；实机阶段仍需增加硬件级保护。

阶段 3 的 `/m20/floor_switch` Action 实现双向 F1↔F2 原子切换：

```text
校验楼层和电梯触发点
  -> 断言 floor-switch hold
  -> 连续确认安全速度与里程计停止
  -> reset 旧 SCAN
  -> 模拟电梯等待
  -> generation CAS 切换地图
  -> 平面传送仿真位姿
  -> reset 新 SCAN
  -> 等待新楼层/新代次的连续新鲜 local cloud
  -> 成功后释放 hold
```

事务启动后的取消、超时或任一步骤失败都会保持 hold。无效请求或机器人不在电梯触发
点时事务不会启动，也不会改变地图。Action 使用 `MultiThreadedExecutor`，使等待服务
或状态期间订阅回调仍能持续更新。

阶段 4 增加 typed 自动任务执行器：

```text
/m20/mission/run
  -> inspection step: /m20/navigation/navigate
  -> elevator step: lobby navigation -> cabin navigation
                    -> /m20/floor_switch
  -> dwell / next step
```

默认任务按配置执行 F1_A、F1_B、E1:F1→F2、F2_A、F2_B。执行器不直接发布 SCAN
目标或操作地图，只组合 `NavigateFloor` 与 `SwitchFloor` Action。

`/m20/mission/control` 提供 pause、resume、stop 和 retry-current。pause 会取消当前
导航、等待子 Action 收口并通过 `/m20/control/mission_hold` 保持零速；resume 从当前
位置重新规划。楼层切换是原子事务，中途拒绝 pause。导航或切层失败进入
`FAULT_HOLD`，故障 retry 会重试当前 step；若目标地图已提交但切层 hold 尚未释放，
floor-switch manager 会执行 `RECOVERING_COMMITTED_TARGET` 恢复路径，而不是错误地
再次递增 generation。

`mission_hold` 与 `floor_switch_hold` 相互独立，任一为 true 时 safety supervisor
立即输出零速度。stop、任务取消和未恢复故障都会保留任务 hold，避免上层任务退出后
底层旧命令恢复。
