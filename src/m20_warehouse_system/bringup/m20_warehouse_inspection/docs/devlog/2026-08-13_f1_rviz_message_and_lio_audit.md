# 2026-08-13 F1 RViz、实机消息包与 Elevator-LIO 审计

## 1. F1 自由导航入口

`f1_scan_rviz.launch.py` 不是多余入口。它用于不启动 MuJoCo 和自动巡检时，快速检查
静态 PCD、原生 SCAN、M20 模型、RViz 运动学后端与安全链路。

本轮在独立 ROS_DOMAIN_ID 下无界面启动 11 个节点，并向
`/move_base_simple/goal` 发送目标；SCAN 成功生成并执行 B-spline，机器人到达目标后回到
`WAIT_TARGET`。故 launch 图本身正常。

点击无响应的根因是入口仍加载旧 `phase2_f1_navigation.rviz`。该文件把 goal 写到已退休
的 `/m20/navigation/scan_goal`，并把点云、占据、膨胀、A-star 与 optimal path 等显示写
到未发布的 `/m20/...` 别名。入口现改为加载 `m20_scan_planner` 安装的
`rviz/default.rviz`，与完整 MuJoCo 链路共用 `/move_base_simple/goal` 和原生 SCAN 可视化
话题。

## 2. deep-robotics-msg 迁移边界

用户确认背部主机的当前消息源码目录为 `src/deep-robotics-msg`。ROS 运行时依据
`package.xml` 中的包名、`.msg` 字段和实际 topic type，不依据源码文件夹名。因此没有在
缺少源码时猜测 Python import 或消息字段。

补全后已确认该仓库修订为
`aa646c1782aa982f384b7e46469723f31696f778`，`package.xml` 声明的 ROS 包名仍是
`drdds`、版本是 `1.1.0`。关键 ABI 结论：

- `MetaType` 是 `uint64 frame_id + builtin_interfaces/Time stamp`；
- `MotionInfoValue.motion_state` 和 `gait_state` 是嵌套消息，后端必须读取
  `motion_state.state` 与 `gait_state.gait`；
- 官方硬急停文档中的旧截图把 `StdMsgInt32` 字段写成 `data`，但当前真实
  `deep-robotics-msg 1.1.0` 源码定义为 `int32 value`；运行代码和预检均以真实安装 ABI
  为准读取 `message.value`。值语义经同一文档确认仍是 `0=未触发、1=已触发`；
- `/NAV_CMD`、`/MOTION_INFO`、`/MOTION_STATE`、`/GAIT`、`/HES_STATUS` 均与背部
  主机已有抓取记录一致；
- 运动话题的实机 QoS 为 `RELIABLE/VOLATILE`，`/HES_STATUS` 为
  `RELIABLE/TRANSIENT_LOCAL`；direct ROS 后端已分开适配；
- Humble 仿真与 Foxy 实机都只启用这一份消息包；历史 `src/drdds` 与 SDK 内
  同名副本均隔离，`colcon list` 只能发现一个 `drdds`。

## 3. 最近 Elevator-LIO 接入的逐项修改与最终修正

审计依据是文件时间、项目文档引用和与锁定官方提交
`1d79af77f3d9747ea57ef52a9b01d326a8ec561a` 的只读对比。最近导航接入没有修改
Elevator-LIO C++ 算法。初版四处配置调整的最终处理如下：

1. `yaml/root_config_m20_navigation.yaml`（初版新增，最终删除）
   - 此前已验证的原始 M20 配置就是 `root_config_m20.yaml`；
   - 它已引用 `robosense_m20.yaml + runtime/mapping.yaml`，平行入口没有新增能力。

2. `yaml/sensors/robosense_m20_navigation.yaml`（初版新增，最终删除）
   - 初版错误地改成 `world/m20_lio_imu/base_link`；
   - 实机开机服务已发布 `map -> base_link`，因此恢复原始
     `lio_world/lio_imu/lio_base_link` 隔离设计，mapping/relocation 都直接引用
     `robosense_m20.yaml`。

3. `yaml/root_config_m20_navigation_relocation.yaml`（2026-08-12 11:49 新增）
   - 直接使用原始 `robosense_m20.yaml`；
   - runtime 从 mapping 切为 relocation；
   - 动机：运行导航时加载已建 PCD 并停止继续积图，避免把建图模式误当定位模式。

4. `yaml/runtime/relocation.yaml`（初版改名，最终撤回）
   - `pcd_load_name` 恢复示例时间戳名；
   - 部署时按实际建图结果修改该字段，不再复制为项目强制固定名。

`yaml/root_config_m20.yaml`、`yaml/sensors/robosense_m20.yaml`、
`yaml/runtime/mapping.yaml` 在最近导航接入中未改，三者作为唯一建图/标定真值。

需要区分：当前导入的 Elevator-LIO 源码本身是相对官方提交已有大量扩展的 fork；这些 C++、
ROS 兼容层、RoboSense 和电梯算法差异在本轮接入前已存在，不能归因于上述四处导航配置。

## 4. 回归结果

- `drdds`、`m20_sdk_deploy`、`m20_locomotion_control`、`m20_mujoco_backend`、
  `m20_warehouse_inspection` 按真实 ABI 重新构建成功；SDK C++ 构建时明确找到
  `drdds 1.1.0`；
- 本轮清缓存重建后回归通过，`m20_locomotion_control` 结果为
  `94 tests, 0 errors, 0 failures, 0 skipped`，总启动包为
  `195 tests, 0 errors, 0 failures, 1 skipped`（cppcheck 2.7 工具自身跳过），两包分别
  `11/11` 与 `21/21` CTest 通过；
- basic_server/direct_ros 两条 Foxy 源码预检均 PASS，含真实包名、版本、嵌套 ABI
  和 `lio_*` relocation 配置检查；
- 单进程运行回放已用真实 `MotionInfo` 嵌套对象执行实际回调，解码为
  `state=17, gait=12290, vx=0.12, vy=-0.03, wz=0.08`；
- direct ROS 节点退出流程改为 ROS context 存活时先发三次零速，再销毁节点与
  shutdown，解决 Ctrl-C 时原有的 `publisher's context is invalid`；
- 修正部署手册中一条会错误隔离新消息包的旧指令：`deep-robotics-msg` 必须保持启用，
  `COLCON_IGNORE` 只用于历史 `src/drdds` 和 SDK 内旧副本；
- LIO 配置策略收敛为原始 `robosense_m20.yaml` 单一标定基线，以及现有 mapping / relocation
  runtime；不再为了导航改名或复制等价 YAML；
- SCAN 输入链路复核通过：定位适配器和原生 `grid_map` 的点云、传感器位姿均使用
  SensorDataQoS；
- 独立 `ROS_DOMAIN_ID=96` 启动 F1 无界面链路，11 个节点正常启动；
- 向 `/move_base_simple/goal` 发布 `(-34.0, 0.0)` 后，SCAN 四次成功规划/重规划，FSM
  回到 `WAIT_TARGET`，最终位姿 `(-34.0021, 0.0, 0.59)`；
- 独立 `ROS_DOMAIN_ID=177` 按主命令的 `cyclonedds_local.xml` 完整启动 MuJoCo 一键链路，
  14 个节点正常启动；官方 SDK 站立稳定后，`backend_ready=true`、`map.ready=true`、
  位姿有效，感知 `ready=true` 且点云持续更新；随后 SIGINT 结束。

注：一次未带项目 CycloneDDS 配置的完整启动因默认 participant 索引不足失败；
按用户主启动环境加载 `cyclonedds_local.xml` 后通过，确认这是测试环境偏差而不是
本轮代码回归。
