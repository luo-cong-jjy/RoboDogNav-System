# M20 机器狗导航与仓库巡检工作空间

本工作空间面向云深处 M20-pro 轮足机器狗，包含两条主要技术路线：

- `m20_nav2_system`： Nav2 + Gazebo 的二维导航、雷达和工厂巡检验证,仿真底层可扩展为Mujoco➕官方强化学习运控ONNX部署包，后续亦可部署到实机。
- `m20_warehouse_system`：融合FAST-Livo2的电梯扩展雷达建图定位，SCAN-Planner、多区域/多楼层任务、安全控制以及 MuJoCo/实机后端。

ROS 包、第三方依赖、数据集和工程文档按用途分开管理，构建产物不属于源码。

## 目录结构

```text
robodog_nav_system/
├── src/                          # 源代码（自己开发的、二次开发、第三方包）
│   ├── m20_nav2_system/          # Nav2方案集成包
│   ├── m20_warehouse_system/     # FAST-Livo2及SCAN-PLANNER方案仓库巡检系统源码栈
│   └── third_party/              # 外部算法、驱动、消息和厂家 SDK
├── tools/                        # 工作区级雷达和网络诊断工具
├── build/                        # colcon 构建的中间编译缓存，cmake 构建目录，目标文件、obj、临时产物
├── install/                      # colcon 安装输出目录，编译后可执行文件、launch、yaml、msg、配置文件最终在这里
├── log/                          # colcon 构建日志，可清理
├── datasets/                     # rosbag、PCD 等测试数据
├── docs/                         # 跨模块实机部署与接口记录
```

## 编译方式
1.工作空间完整普通编译：
   ```bash
   colcon build

2.限制编译时CPU线程，防止编译崩溃
    例如只开2线程：
    ```bash
    colcon build --parallel-workers 2
    表示：最多两个 package 同时编译。


## 可清理目录
以下内容可通过重新构建或重新测试生成：
```text
build/
install/
log/
```

删除 `install/` 后，运行 ROS 节点前必须重新执行 `colcon build` 并重新加载，`install/setup.bash`。`datasets/`、`docs/`不属于普通构建缓存，为用户自创建数据。
