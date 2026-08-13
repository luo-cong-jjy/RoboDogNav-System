# 2026-08-10 Foxy/LIO 接口适配后的仿真回归

> 历史记录：其中“direct ROS 缺少高层 DrDDS 消息”的阻塞已由 2026-08-11 完整开发指南
> 核对和双后端实现解除；当前状态见
> [`2026-08-11_factory_dual_transport_and_motion_feedback.md`](2026-08-11_factory_dual_transport_and_motion_feedback.md)。

## 验证边界

本轮只验证新增真机定位/速度接口、Foxy 兼容修正和 hardware launch 没有破坏原有
Humble + MuJoCo 仿真系统。`Elevator-LIO` 不属于本机仿真编译闭包，本轮只审查其源码
发布的话题、消息内容和坐标系；其 Foxy 构建、双雷达数据与实时性能留在背部 x86 主机
验证。

## 静态、构建与测试结果

| 项目 | 结果 |
| --- | --- |
| `m20_warehouse_inspection` 完整依赖构建 | 22/22 包成功 |
| 项目包测试 | 543 tests，0 error，0 failure，1 skipped |
| 新定位适配器针对性测试 | 13/13 通过 |
| Foxy 源码预检（`basic_server`） | PASS |
| direct ROS 预检 | 按设计阻断，缺少 `Gait/MotionInfo/MotionState/NavCmd` |
| `Elevator-LIO` 本机编译 | 未执行，按部署边界留给 Foxy 目标机 |

## 无界面 MuJoCo 两目标回归

启动 `inspection_mission_mujoco.launch.py`，保持原仿真感知、SCAN、安全仲裁、本地 M20
策略与 MuJoCo 执行链。测试目标为从约 `(-37, 0)` 前进到 `(-34.5, 0)`，再不强制掉头
返回 `(-37, 0)`。

| 指标 | 前进 2.5 m | 反向返回 2.5 m |
| --- | ---: | ---: |
| 任务成功 | true | true |
| 用时 | 9.54 s | 11.08 s |
| 终点误差 | 0.175 m | 0.197 m |
| 最大横向偏差 | 0.022 m | 0.052 m |
| SCAN 轨迹消息 | 7 | 6 |
| collision stop 样本 | 0 | 0 |
| recovery 事件 | 0 | 0 |
| 障碍物接触事件 | 0 | 0 |
| backend fault 样本 | 0 | 0 |

返回段 96.3% 的运动样本采用 `REVERSE`，没有为了反向目标在窄空间强制掉头。两段均由
导航探针判定成功，证明真机适配新增代码没有改变默认仿真执行后端或破坏导航闭环。

## 结论

当前 Humble 仿真模式回归正常。该结论不代表 `Elevator-LIO` 已在本机或 Foxy 目标机
编译成功，也不代表 RoboSense、AOS/basic_server 网络和实机运动已经验收；这些是后续
目标机分级联调项。
