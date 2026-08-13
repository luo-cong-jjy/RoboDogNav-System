# 2026-08-10 官方运动后端抽象与仿真回归报告

## 目的

确认新增 M20 出厂 `basic_server` 运动后端、真实数据源参数和 hardware launch 后，原有
SCAN + 安全仲裁 + 本地 ONNX + MuJoCo 仿真链没有被误切换或破坏。本轮没有连接实物。

## 静态与构建验证

| 项目 | 结果 |
| --- | --- |
| 完整依赖构建 | 22/22 包成功 |
| 四个核心包源代码测试 | 221/221 通过 |
| 新协议/profile/launch 针对性测试 | 27/27 通过 |
| locomotion + integration flake8 | 61 个文件无问题 |
| 新运动包 copyright/PEP257 | 无问题 |
| hardware launch 参数展开 | 成功 |

构建入口：

```bash
colcon build --symlink-install \
  --packages-up-to m20_warehouse_inspection \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

hardware launch 的静态展开确认了真机默认锁：

- `command_ownership_confirmed=false`；
- `auto_enable_motion=false`；
- 实位姿 `/m20/localization/body_pose`；
- 实点云 `/m20/sensing/cloud_map`；
- 实测速度 `/m20/locomotion/measured_twist`。

## MuJoCo 动态回归

### 条件

- ROS_DOMAIN_ID：189，与现有会话隔离；
- 启动：`inspection_mission_mujoco.launch.py use_rviz:=false
  use_mujoco_viewer:=false`；
- 场景：默认 `dense_four_corner_system.yaml`；
- 路径：原有仿真 PCD 感知、SCAN、grid route、安全仲裁、本地 M20 ONNX 和 MuJoCo；
- 目标：从约 `(-37, 0)` 到 `(-34.5, 0)`，再回到 `(-37, 0)`；
- 成功条件：终点误差不大于 0.25 m 且低速稳定 1 s。

### 结果

| 指标 | 前进 2.5 m | 回到起点 |
| --- | ---: | ---: |
| 成功 | true | true |
| 用时 | 9.09 s | 12.49 s |
| 终点误差 | 0.171 m | 0.207 m |
| 最大横向偏离 | 0.014 m | 0.050 m |
| SCAN 轨迹消息 | 6 | 5 |
| collision stop 样本 | 0 | 0 |
| 障碍接触事件 | 0 | 0 |
| recovery 事件 | 0 | 0 |
| backend fault 样本 | 0 | 0 |

导航网关两段均报告 `SUCCEEDED`，仿真仍启动 `rl_deploy_cmdvel` 与
`m20_mujoco_backend`，没有误启动 `m20_basic_server_backend`，说明执行后端隔离有效。

## 已知非阻塞告警

- 启动早期及长时间空闲时出现过瞬时 `ODOM_STALE`，安全链自动归零并恢复；两段任务内
  没有 collision stop 或 backend fault。该告警在本次抽象前的 MuJoCo 链也可能因调度
  抖动出现，后续耐久测试应继续统计。
- Ctrl-C 关停时出现一次 ROS 2 Humble `rclpy Future.__del__` 清理告警，所有 17 个进程
  随后均干净退出；不属于运行期导航故障。
- shell 环境仍带有若干已隔离旧包的失效 `AMENT_PREFIX_PATH/CMAKE_PREFIX_PATH` 提示。
  新终端只 source `/opt/ros/humble` 和当前 `install/setup.bash` 可避免继续叠加；本轮未
  擅自修改用户 shell 配置。

## 结论范围

本轮证明“新增真机执行后端与数据源参数化”没有破坏默认 MuJoCo 功能。它没有证明真实
M20 的 basic_server 固件版本、速度死区、符号、转弯扫掠、制动、定位、雷达或硬件急停
已经可用；这些仍必须按迁移文档分级验收。
