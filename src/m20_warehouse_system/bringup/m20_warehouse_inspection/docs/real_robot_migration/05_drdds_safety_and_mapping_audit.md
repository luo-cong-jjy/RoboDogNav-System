# 实物部署准备 05：drdds、急停与建图审计结论

> 审计日期：2026-08-12  
> 机器人目标：山猫 M20-PRO，背部 x86 Ubuntu 20.04 / ROS 2 Foxy  
> 结论性质：源码与手册审计已完成；目标机 QoS、固件行为和物理测试仍待现场验收

## 1. drdds 真值与差异

本次以 `src/drdds-背部主机当前版` 作为消息 ABI 真值，因为它是最近实际部署在背部主机的
v1.2.0 包；开发指南截图可能来自较旧版本，只用于理解语义。

修正前活动包有三处实质差异：

- `MetaType.msg` 错用自定义 `Timestamp timestamp`，实机版是
  `builtin_interfaces/Time stamp`；
- `MotionInfoValue.msg` 错用嵌套 `MotionStateValue/GaitValue`，实机版是扁平
  `int32 state/uint32 gait`；
- 活动包多出实机版不存在的 `Timestamp.msg`，且缺少 `builtin_interfaces` 依赖。

现在活动 `src/drdds` 与背部基准共有 25 个 `.msg`；忽略注释和空白后，消息集合及字段
完全一致，规范化集合 SHA-256 均为：

```text
443389e6e52e2bbca3604afc639fc5a6d2db23ff42f2ff98bf5d8cbdf943a5e2
```

背部基准和 SDK 内旧 drdds 均用 `COLCON_IGNORE` 隔离，只编译顶层活动包。Foxy 预检会在
每次发布时重做全部消息对比；SDK/MuJoCo 中的 `header.stamp` 消费者及锁定补丁也已同步。

## 2. 官方安全事实与当前实现

根据《硬急停》《运动控制（ROS2）》《运动控制（basic-server协议）》和《错误码与异常
处理》：

- 尾部红色旋转急停是人工硬件装置，触发后切断关节电机动力；软件不能替代、触发或
  释放它；
- basic_server 的 `BasicStatus.HES` 和 ROS `/HES_STATUS` 可观测状态；后者约 1 Hz；
- ROS 速度命令应保持 20 Hz，约 500 ms 不刷新会自动减速/停车；
- 运动状态 17 才是 RL 控制；切步态需停稳；
- 设备温度、电流、压力、关节角、姿态、电池和计算单元错误必须进入故障处理。

本项目两条运动后端都默认不自动 enable，并实现：

1. 300 ms 本地命令超时，短于厂家约 500 ms 看门狗；
2. `basic_server` 必须收到新鲜 BasicStatus 且 HES=0；direct ROS 必须收到新鲜
   `/HES_STATUS.value=0`；
3. HES 触发时立即清零、禁用并锁存；物理释放后仍不自动恢复；
4. 操作员检查后必须调用 `enable_motion false` 清确认锁，再显式 `true`；
5. basic_server 异步设备错误禁用并闭锁运动，enable/disable 不清锁，必须排障后重启；
6. 后端未 ready/fault 非空时 locomotion manager 不放行速度。

这些措施降低软件误恢复风险，但不能“证明实机安全”。尾部急停电气动作、制动距离、
遥控接管、断网停车和设备错误报文必须在支撑架和空场按 04 手册验证。

## 3. 地图链路结论

项目正式链路不使用 PGM/YAML：

```text
双雷达 + IMU
  -> Elevator-LIO mapping
  -> binary <timestamp>_scans.pcd
  -> 人工静态清理
  -> PCL 转 ASCII
  -> m20_import_site_pcd
  -> static_map.pcd + static_map.json
  -> SCAN 静态地图
```

Elevator-LIO 只在 mapping 模式正常退出时合并保存地图，实际输出目录由源码编译宏确定为
`src/Elevator-LIO/PCD/`。导航定位使用 relocation profile 加载稳定名
`m20_pao_f1_scans.pcd`，且要求靠近原地图起点冷启动；它不提供任意初始位姿全局定位。

原厂 `drmap mapping/stop_mapping/pack/unpack/apply` 属于 M20-PRO 厂家自带建图定位栈，是
一条可选替代方案。当前 SCAN+Elevator-LIO 项目不调用这些命令，也不把厂商地图包和 LIO
PCD 混用。若以后切到厂商定位，必须另建后端适配 `/ODOM`、`/LOCATION_STATUS`、
`/initialpose` 及地图版本，而不是同时启动两个 `world -> base_link` 来源。

## 4. 本轮新增的 fail-closed 边界

- `config/sites/m20_pao_f1_template.yaml`：允许真正单层硬件配置，非零 PCD 平移被拒绝；
- `m20_import_site_pcd`：规范化真实 PCD并生成来源/输出 SHA 元数据，默认拒绝覆盖；
- `validate_map_assets`：仿真资产继续走严格生成器校验，真实资产走 surveyed 校验；
- 硬件 launch 默认指向缺少实场资产的模板，因此未建图时安全失败，不会误加载仿真地图；
- `root_config_m20_navigation_relocation.yaml`：明确加载 F1 稳定地图名。

## 5. 仍需现场提供/确认

- 背部主机 IP、SSH 用户、实际工作空间路径；
- AOS IP、固件/basic_server 版本、`ROS_DOMAIN_ID`；
- 已验证双 RoboSense 驱动包名、版本和精确 launch 命令；
- 前后雷达/IMU 外参、时间同步方式；
- 目标机 `ros2 interface show` 五个关键消息及 `/HES_STATUS`、`/MOTION_INFO` QoS；
- 停止机载 planner、自动充电和其他速度源的厂家步骤；
- F1 建图原点、可重复启动标记、地图范围、审核后的目标点；
- 物理急停、软急停和遥控接管的现场负责人及官方恢复流程。

这些数据缺失不影响源码构建，但会分别阻塞 LIO 放行、direct ROS 放行或任何实机运动。
