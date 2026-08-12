# 2026-08-11 M20 出厂双运动链路与速度反馈验证

## 手册结论

本轮依据 `src/third_party/山猫M20 开发指南/山猫M20 开发指南/山猫M20开发者文档`
中的“运动控制（ROS2）”“ROS2-DDS 接口总览”“ROS2-DDS 话题速查”逐字段核对：

- `/MOTION_INFO` 为机器人到客户端的 20 Hz 状态，`vel_x`、`vel_y`、`vel_yaw`
  分别是机体系前向、左向和逆时针偏航实测速率；它是直连 ROS 模式的闭环速度源。
- `/NAV_CMD` 使用 `x_vel`、`y_vel`、`yaw_vel`，单位和符号与上述反馈一致。系统以
  20 Hz 下发，并以 300 ms 本地超时保持在固件文档的 500 ms 失联停车窗口内。
- 只有运动状态 17 允许运动；直连链显式执行站立状态 1、RL 状态 17，再在静止时切换
  到自主导航建议步态 `0x3002`。
- 厂家将 `basic_server` 作为更简单且自动处理状态推进的优先入口；完整开发指南也允许
  外接 Foxy/Humble/Jazzy 主机通过 DrDDS ROS2 话题做自定义导航。

因此生产默认保持 `basic_server`，同时提供 `factory_transport:=direct_ros`。两条链路在
同一启动中互斥，并统一输出 `/m20/locomotion/measured_twist`：前者来自 TCP
`MotionStatus`，后者来自 `/MOTION_INFO`，规划、安全和任务层不感知传输差异。

## 源码与独立工作空间

- 工作区只启用 `src/drdds/drdds`，它合并高层运动消息和 SDK 低层消息；SDK 自带旧同名
  副本由 `COLCON_IGNORE` 隔离。
- 独立工作空间准备脚本会复制完整 DrDDS，再隔离新克隆 SDK 的旧副本。
- 使用锁定的三个本地依赖仓库实际准备了隔离源树，`colcon list` 得到 22 个包且活动
  `drdds` 数量为 1。
- 在该隔离源树中单独构建 `drdds`、`m20_locomotion_control`、`m20_sdk_deploy`，3/3
  成功，证明锁定补丁与完整时间戳模式可从干净 revision 重建。

## 静态、构建和消息闭环

| 验证 | 结果 |
| --- | --- |
| 主仿真闭包构建 | 22/22 包成功；未编译 `Elevator-LIO` |
| 主闭包测试记录 | 557 tests，0 error，0 failure，1 skipped |
| SCAN launch 测试 | 将只读默认日志目录改到 `/tmp` 后 2/2 通过 |
| Foxy 源码预检 | `basic_server`、`direct_ros` 均 PASS |
| 直连 DrDDS schema | 与指南逐字段一致 |
| 直连 ROS 本机回环闭环 | enable、ready、实测 Twist、非零 `/NAV_CMD` 均通过 |

直连闭环使用 `ROS_LOCALHOST_ONLY=1`，没有接入真实机器人。模拟 `/MOTION_INFO` 的
`vel_x=0.12`、`vel_y=-0.03`、`vel_yaw=0.08` 被转换成
`/m20/locomotion/measured_twist`；状态 17、步态 12290 就绪后，上层输入
`vx=0.20`、`wz=0.40`，采集到的 `/NAV_CMD` 最大值分别为 0.20 和 0.40。

## MuJoCo 两目标动态回归

完整 SCAN、点云、安全仲裁、本地 ONNX 策略和 MuJoCo 链在本机回环域运行，目标从约
`(-37, 0)` 到 `(-34.5, 0)`，再返回 `(-37, 0)`。

| 指标 | 前进 2.5 m | 返回 2.5 m |
| --- | ---: | ---: |
| 成功 | true | true |
| 用时 | 9.77 s | 11.08 s |
| 终点误差 | 0.181 m | 0.195 m |
| 最大横向偏差 | 0.013 m | 0.050 m |
| 障碍接触事件 | 0 | 0 |
| 后端故障样本 | 0 | 0 |

## 尚待真机验证

源码完成不等于实物验收。背部 Foxy 主机仍需确认机器人软件版本至少 V1.1.7、
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、相同 `ROS_DOMAIN_ID`、实际话题 QoS、速度符号、
`basic_server` 端口/状态字段、命令源唯一所有权和硬件急停。`Elevator-LIO` 本轮仅审查
接口，不在 Humble 仿真主机编译。
