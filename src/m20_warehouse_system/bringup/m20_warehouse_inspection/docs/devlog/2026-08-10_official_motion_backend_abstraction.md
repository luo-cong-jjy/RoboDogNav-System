# 2026-08-10 官方运动后端抽象开发记录

## 目标

根据本地 M20 Pro 官方技术手册，判断 MuJoCo ONNX 运控能否直接迁移到实机，并建立不
破坏现有仿真的真机运动后端与数据源切换边界。

## 主要判断

- MuJoCo 路径在本地执行 ONNX 并输出 16 路关节指令；出厂运动层接收机体 X/Y/Yaw 并
  在 AOS 内完成轮腿协调。两者只能在安全 Twist 边界做功能替换。
- 当前 vendored `drdds` 缺少新版导航运动消息，第一版选择文档完整的 basic_server，
  保留以后增加 ROS 2 direct transport 的接口位置。
- MotionStatus 是速度反馈而不是全局定位；真机定位/TF 与雷达链必须独立接入。

## 代码变更

- `m20_locomotion_control` 新增 basic_server APDU codec、真实后端节点、配置与 launch。
- 新增真机敏捷平地 profile，避免复用 MuJoCo ONNX 策略的实测漂移/倒车参数。
- navigation adapter 支持 Odometry 或 TwistStamped 两种速度反馈源。
- SCAN、导航网关、安全、碰撞保护、切层管理的位姿/点云话题由 launch 参数注入。
- `m20_warehouse_inspection` 新增 guarded hardware launch；默认不确认命令所有权、不自动
  使能运动，并在真实定位/点云缺失时保持闭锁。

## 安全决策

- basic_server 20 Hz 发送，系统命令 300 ms 超时，短于官方约 500 ms 看门狗。
- enable 必须显式调用，且 `command_ownership_confirmed` 必须为 true。
- 急停发送零速和官方软急停；断网、状态超时、非 RL 状态或步态不一致不 ready。
- 官方非零速度以下默认归零，不在碰撞预测之后抬升指令。

## 验证

- 完整依赖构建：22/22 包成功。
- 运动、导航、巡检与集成包源代码测试：221 tests passed。
- 新增协议、能力 profile、launch 静态契约针对性复验：27 tests passed。
- 无界面 MuJoCo 2.5 m 前进/回程：2/2 成功，终点误差分别为 0.171 m、0.207 m；
  collision stop、障碍接触和 backend fault 均为 0。
- 独立报告：
  `docs/test_reports/2026-08-10_official_motion_backend_simulation_regression.md`。

## 未完成/阻塞

- 未连接实体机器狗，不能确认固件版本、真实速度死区、转弯扫掠和停车距离。
- 尚缺真实定位、雷达点云转换、时间同步、硬件急停/遥控接管和电梯 provider。
