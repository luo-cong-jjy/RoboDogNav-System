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

完整系统的 `m20_collision_guard` 对 active occupancy 使用前后双圆检查滚动化后的
候选命令。正常 native-SCAN profile 为 0.25 m 机身半径加 0.05 m 余量、0.70 s
前视；0.60 m 膨胀的后备栅格路线使用独立 robust profile。地图或里程计未就绪时
默认断言停车。该保护是独立于 SCAN 内部 GridMap 的最后一道静态地图导航保护；
实机阶段仍需增加硬件级保护。

保护器区分保守安全栅格和不含额外余量的 hard-body 栅格。只有保守离散外壳误命中且
整条 hard-body 扫掠自由时，才允许有界 `RASTER_SHELL_ESCAPE`；共享地图边缘的
`MAP_EDGE_INWARD_RECOVERY` 也必须经过完整直线扫掠验证。实体机身占用仍硬停车，
两类恢复都不能使用纯偏航、横移或带偏航倒车。

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

后备路线还通过 `/m20/control/route_segment_hold` 请求段间锁存停车。该保持与上述
任务/切层保持同级，立即输出零速并冻结旧 SCAN 轨迹；只有下一段目标已发布且里程计
完成连续停稳确认后才释放。
