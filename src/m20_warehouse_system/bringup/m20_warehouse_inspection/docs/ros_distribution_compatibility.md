# ROS 2 Humble/Foxy 兼容性清单

日期：2026-07-27  
主验证发行版：ROS 2 Humble  
实机候选发行版：ROS 2 Foxy

## 1. 结论

当前项目集合在 Ubuntu 22.04 / Humble 已完成独立构建、自动测试和运行时回归。
已确认实际部署主机是 M20 背部 x86、Ubuntu 20.04 / Foxy，因此推荐部署从原来的
“双主机候选”调整为“Foxy 单机原生构建”。Foxy 目前仍只完成源码与接口静态审计，
不能标记为“已构建”或“已运行”：

```text
Humble clean build/runtime        VERIFIED
Python 3.8 import-risk audit      STATIC PASS
common message/interface types    STATIC PASS
Elevator-LIO Foxy source profile  STATIC PASS / TARGET NOT RUN
Foxy native build                NOT RUN
Foxy runtime/DDS                  NOT RUN
SCAN Foxy backport                REQUIRES VALIDATION
```

ROS 2 Foxy 已结束官方支持，当前 Jammy/Humble 主机的 rosdistro 索引会跳过 Foxy。不要
为了得到一个虚假的绿色结果，在 Jammy 环境中把 `ROS_DISTRO` 改为 `foxy` 后直接复用
Humble 二进制包。

## 2. 已保持在共同边界内的内容

### 2.1 自定义接口

`m20_warehouse_interfaces` 使用的都是 Foxy/Humble 共同支持的 rosidl 类型：

- `bool`、整数、浮点、字符串；
- `geometry_msgs/PoseStamped`；
- 普通 msg、srv、action；
- 不使用 type adaptation、loaned message 或发行版特有接口。

因此 floor/generation、导航、任务和切层的 wire contract 可作为跨主机稳定边界。
跨发行版 DDS 是否能直接互通仍必须用目标 RMW 实测，不能只由消息定义推断。

### 2.2 Python 语法

七个项目包的 51 个 Python 文件已用 Python 3.8 grammar 解析通过。源码没有使用：

- `match/case`；
- Python 3.9 才支持的内建容器泛型；
- Python 3.10 union `A | B`；
- Python 3.11 专有标准库接口。

这只证明语法可加载，不证明 Foxy 版 `rclpy` 的每个行为完全一致。

### 2.3 ROS 常用 API

当前主要 API 在 Foxy 已有对应概念：

- `rclpy` publisher/subscription/service/action；
- `MultiThreadedExecutor` 和 callback group；
- reliable、transient-local QoS；
- `robot_state_publisher`、`xacro`、TF2；
- `rclcpp` parameter、timer、publisher/subscription；
- ament CMake 和 rosidl generators。

项目没有使用 Humble lifecycle component、composition container 或 Nav2 专有 API。

## 3. 必须在 Foxy 环境验证的风险项

| 项目 | Humble | Foxy 风险与动作 |
|---|---|---|
| SCAN-Planner ROS 2 port | 独立构建/运行通过 | 在 Ubuntu 20.04/GCC 9 重新编译固定 commit 与 CPU patch |
| `rclpy` Action cancel/result | 运行回归通过 | 核对 Foxy goal/cancel 回调时序并重跑故障注入 |
| `sensor_msgs_py` PointCloud2 | 10 Hz 运行通过 | 核对 Foxy 包版本和 NumPy 布局 |
| launch event shutdown | 自动测试通过 | 核对 Foxy `OnProcessExit/Shutdown` 行为 |
| static TF CLI 参数 | Humble 通过 | Foxy 必要时改用其兼容的位置参数形式 |
| CycloneDDS 配置 | Humble 通过 | 目标机 RMW/版本可能不同，重新做 participant 与 QoS 验证 |
| C++17 | GCC 11 通过 | Ubuntu 20.04 GCC 9 支持，但必须全闭包重编译 |
| rosdep | Jammy 闭包通过 | 使用 Focal/Foxy 自身索引重新解析，不能沿用 Jammy 结果 |

尤其不能把 Humble 生成的 `build/`、`install/` 或 Python generated interface 复制到
Foxy。Foxy 验证必须从同一锁定源码重新构建。

## 4. 推荐部署策略

现场已指定背部 x86 为 Ubuntu 20.04 / Foxy，因此主策略改为同机运行 Elevator-LIO、SCAN、
地图、任务、安全与运动 adapter：

```text
M20 back-mounted Foxy x86
  RoboSense + IMU -> Elevator-LIO
  map + SCAN + mission + safety
  /m20/locomotion/cmd_vel_sdk
  factory motion adapter -> AOS
```

这降低了跨发行版 DDS 的不确定性，但扩大了 Foxy 原生验证闭包。`basic_server` 不依赖
高层 drdds，因此仍是默认真实执行传输；direct ROS 已按完整指南补齐消息和后端，但必须
在目标机使用 Fast DDS 并核对实际 QoS 后放行。详细链路见
`real_robot_migration/03_foxy_x86_elevator_lio_integration.md`。

## 5. Foxy 原生验收矩阵

在 Ubuntu 20.04 / Foxy 容器或目标机上：

1. 使用同一 `workspace_lock.yaml` 准备全新源码树。
2. 针对 Focal 运行 `rosdep install`。
3. 构建 Foxy 实机 closure，不复用 Humble 产物；Elevator-LIO 建议关闭 sim、RViz plugin
   和 Livox 支持，只保留双 RoboSense 路径。
4. 运行全部 unit/launch tests。
5. 检查所有自定义 msg/srv/action。
6. 运行 quick typed navigation。
7. 运行 F1↔F2 两次切层。
8. 运行一次完整任务。
9. 重跑 cancel、stop、FAULT_HOLD retry。
10. 连接 SDK 前增加速度缩放、watchdog、物理急停和人工接管验收。

只有以上矩阵通过后，才能把 Foxy 状态从 `NOT RUN` 改为 `VERIFIED`。
