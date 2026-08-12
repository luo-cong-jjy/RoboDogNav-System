# 0.7.0 开发记录：官方 M20 与原生 SCAN 链路校正

日期：2026-07-27  
版本：0.7.0  
范围：官方 M20 RViz 模型、SCAN 实现归属、原生规划参数、实时点云和 RViz 调试显示  
不在本次范围：Gazebo 动力学/雷达、真实电梯、实机 SDK

## 1. 问题复核

桌面验收发现两类偏差：

1. 集成模型的 xacro 在官方 M20 运动树之外增加了 `lidar_link` 和 IMU，且用户观察到
   机身与腿部显示异常。
2. 系统实际启动了 `m20_scan_navigation` 中复制的 SCAN C++ 核心，默认目标先经过
   栅格 A* 和顺序短子目标；RViz 左侧面板、实时 sensor cloud、GridMap occupancy、
   搜索过程和优化轨迹显示没有完整保留。

进一步参数对比确认，仓库 profile 还把上游 SCAN 的 GridMap 分辨率
`0.05 -> 0.10 m`、规划视野 `7.5 -> 2.5 m`、规划速度上限
`0.75 -> 0.40 m/s`。即使不考虑 A*，这些值也会改变轨迹形态。

## 2. 模型校正

模型真值固定为：

```text
src/third_party/deep_robotics_model/M20/M20_urdf/urdf/M20.urdf
revision 6113c62da96295e8d53abbc079af5296bf4649f8
SHA-256 8202cee50ea319c4c238d0402d9291fa56138ca8e2ba795a3747738bd194d094
```

`m20_official_description` 中：

- `urdf/vendor/M20.urdf` 与上述官方文件逐字节相同，SHA-256 相同。
- 17 个 STL 均与官方目录逐字节相同。
- ROS-facing `m20_official.urdf` 只移除 MuJoCo 专用 `compiler` 元素，并将相对 mesh
  URI 改为 `package://m20_official_description/...`。
- 兼容 xacro 不再增加雷达、IMU、joint 或几何。
- 仓库的 F1、多区域和单模型显示入口直接读取 ROS-facing 官方 URDF。

`check_urdf` 结果为 17 link、16 joint：`base_link` 下有四条独立但完整连接的
`hipx -> hipy -> knee -> wheel` 链。运行时 `/joint_states` 也发布同一组 16 个官方
关节名。

## 3. SCAN 实现归属

当前唯一 C++ 核心为：

```text
m20_scan_planner
  scan_planner_node
  closed_loop_controller
  GridMap / search / B-spline / FSM
  /m20/navigation/reset
```

`m20_scan_navigation` 只保留：

```text
typed navigation gateway
optional grid-route adapter
bringup and parameter profiles
```

其 CMake 不再生成 planner/controller，早期复制且未参与构建的 7 个 C++ header/source
也已删除。楼层事务所需 reset 和近终点退出位于唯一的 `m20_scan_planner` FSM 中。

## 4. 两个明确的导航 profile

默认：

```text
use_grid_route:=false
/m20/navigation/scan_goal -> native SCAN -> controller -> cmd_vel_raw
config/f1_planner_native.yaml
```

原生配置逐项保持固定上游的核心轨迹参数：0.05 m GridMap、7.5 m 规划视野、
0.75 m/s 规划上限和原优化权重。仅覆盖 M20 的 0.59 m 机身高度、碰撞几何、工程
frame 和 0.20 m 终点退出。控制器仍限速到 0.40 m/s，安全 supervisor 的最终边界为
0.45 m/s，规划参数不会绕过安全层。

可选兼容：

```text
use_grid_route:=true
/m20/navigation/goal_pose -> inflated A* -> short scan_goal -> SCAN
config/f1_planner.yaml
```

该 profile 保留阶段 2～5 已验收的 0.10 m / 2.5 m / 0.40 m/s 仓库参数，适合需要
静态栅格长距离引导的任务。两种模式不再隐式混用。

## 5. RViz 与实时感知

`phase2_f1_navigation.rviz` 恢复：

- Displays、Selection、Tool Properties 和 Views dock。
- `/m20/sensing/sensor_cloud` 实时点云。
- occupancy、inflated occupancy、sliding-map bounding box。
- global/init/rebound-A*/optimized path、goal、碰撞包络和 TF。
- 默认 `2D Goal Pose -> /m20/navigation/scan_goal`。

generation-aware sensing 同时发布 planner stream `/m20/sensing/local_cloud` 和
RViz stream `/m20/sensing/sensor_cloud`。实际 QoS 端点检查确认 occupancy 发布/
订阅均为 Best Effort，optimized trajectory 发布/订阅均为 Reliable。

## 6. 独立边界

项目源码边界由 7 个增至 8 个，加入 `m20_scan_planner`。固定第三方算法与独立演示
依赖计入后，工具实际解析的构建闭包为 18 包。准备脚本使用本地固定 revision 在
`/tmp/m20_isolated_model_scan_20260727` 成功生成临时工作空间，revision、patch hash、
8 个项目包和 18 包拓扑均通过检查；验证结束后已删除该临时副本。

本会话受控执行器中，`colcon` 三次在 CMake 已输出成功配置/编译/安装后无法返回父
进程；同样源码在当前工作区用逐包单线程 CMake 安装全部通过。因此本记录不把新的
18/18 fresh colcon 标为 PASS；应在普通终端按 `docs/isolated_workspace.md` 再执行
一次归档级 clean build。阶段 5 的 13/13 是 0.6.0 历史结果。

## 7. 主要文件

```text
m20_official_description/{urdf,xacro,launch,test}
m20_scan_planner/{CMakeLists.txt,package.xml,src,include,launch,README.md}
m20_scan_navigation/{CMakeLists.txt,package.xml,launch,config,test,README.md}
m20_multifloor_map/m20_multifloor_map/dynamic_local_sensing_node.py
m20_warehouse_inspection/{launch,rviz,config,test,docs}
```

验收数据见 `docs/test_reports/2026-07-27_model_scan_correction.md`。
