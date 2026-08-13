# 高密度四角巡检场景副本开发记录

日期：2026-07-28

## 目标

在不覆盖已验证 `flat_multifloor_system.yaml` 的前提下，复制并扩展一套独立场景：

- 每个场景提高障碍物密度；
- 独立障碍物之间的边界距离不小于 0.7 m；
- F1、F2 各配置左下、右下、右上、左上四个巡检点；
- 巡检顺序固定为左下、右下、右上、左上；
- F2 完成四点巡检后回到共享原点，反向切回 F1，再返回整个流程的初始起点；
- 保持 SCAN 原生规划、PCD 切换和无位姿传送机制不变。

## 隔离方式

新增的系统 profile 为：

```text
config/dense_four_corner_system.yaml
```

其资源与基线分离：

```text
maps/dense_four_corner/scene_1/
maps/dense_four_corner/scene_2/
launch/dense_four_corner_mission_rviz.launch.py
mission.id: dense_four_corner_patrol
```

旧 profile 继续是默认入口；其 PCD 和 PGM 哈希在本次修改前后保持完全一致。

## 地图生成器修改

原生成器把障碍物分离值 `0.08` 写死在 `_place_obstacles()` 中。本次增加系统配置项：

```yaml
map_generation:
  minimum_obstacle_spacing: 0.70
```

放置候选障碍物时，先将候选矩形按该值向四周扩张，再拒绝与既有障碍物相交的候选。
该规则比纯欧氏距离更保守：只要候选被接受，其矩形边界欧氏距离必然不小于配置值。

资产元数据新增 `minimum_obstacle_spacing`，资产校验器会执行两级检查：

1. 元数据值必须与系统配置一致；
2. 每一对独立障碍物必须满足配置的最小分离值。

当前 profile 不生成连体障碍物组，因此 246 个障碍物全部按独立障碍物检查。后续若
加入货架组合或连体障碍，应为同组构件增加明确的 group ID，校验器只豁免同组内部。

为保证旧场景不被重排，基线 profile 显式写入原值 `0.08`。重新生成后旧资产哈希为：

```text
F1 PCD 1763a3ceefba9e7a480dc53077f822532e51aed841b707d4f7064b4f89b83054
F2 PCD 34ea9a1158dbc13a37a6150892a51d48f05da92aa71ccdfd0e980411e3fe7f8b
F1 PGM 7e12ff6e847c071ead9716789ff24e1533f1a73a7bdf224b6da6cee7b00dda2a
F2 PGM 60dc019651919dae883f1aee0e6de31ca3925ccb179aa4683d81478eeb82ce22
```

## 新场景参数

两个区域仍为相邻的 40 m × 40 m 平面：

```text
F1: x=[-40, 0],  y=[-20, 20]
F2: x=[0, 40],   y=[-20, 20]
共享切换点: (0, 0)
```

障碍物从每场景 180 个增加到 246 个。其中 240 个为确定性随机障碍物，另外 6 个
为配置中显式命名的 1 m × 1 m 过道固定障碍物。F2 继续通过 `replica_of: F1`
精确复制 F1 内部障碍布局，只把门洞放在相邻的另一侧边界。

生成结果：

| 场景 | 随机/固定/合计 | 点数 | 最近障碍物边界距离 |
|---|---:|---:|---:|
| F1 | 240 / 6 / 246 | 186693 | 0.702172 m |
| F2 | 240 / 6 / 246 | 186693 | 0.702172 m |

PCD 哈希：

```text
F1 2feed3a640bf81a8c11a74f397ee8120a355c85994ecfa4f638d818e90b15876
F2 c1b7817061ed2ed45ddfee9a88ced1c068e2a10e86aa9ddf6e2e341a9c70c6ea
```

六个固定障碍物分别位于下侧、右侧和上侧巡检过道。第一次试验把障碍物直接放在
巡检中心线上，碰撞保护在机器狗接近首个障碍物时按设计安全停车，但当前原生 SCAN
链路未形成稳定绕行。最终改为在中心线两侧交错侵入过道边缘，固定障碍物边缘与
巡检中心线保留 0.8 m 净空，既保留真实的过道静态障碍显示和感知输入，也避免把
整条预定巡检腿正面封死。

生成器先放置配置中的固定矩形，再放置随机矩形；二者使用同一套边界、保护区和
0.7 m 分离校验。资产 JSON 同时记录 `fixed_obstacle_count` 与每个固定障碍物的
名称、中心和尺寸，后续实物场景可直接替换固定布局而无需修改生成代码。

## 巡检点与顺序

F1：

```text
1 (-36, -16) 左下
2 ( -4, -16) 右下
3 ( -4,  16) 右上
4 (-36,  16) 左上
```

F2：

```text
1 ( 4, -16) 左下
2 (36, -16) 右下
3 (36,  16) 右上
4 ( 4,  16) 左上
```

完整任务为 11 步：

```text
F1 四角 -> E1:F1->F2 -> F2 四角 -> E1:F2->F1
-> MISSION_START_RETURN(-37, 0)
```

地图生成时自动把这些 SCAN 直达任务段加入保护走廊，避免随机障碍物覆盖配置点或
切断预定巡检腿。

## 场景 1 出口停住问题

现场日志证明第五步已经在共享边中点 `(0, 0)` 完成 F1→F2 地图提交，active floor
变为 F2、generation 变为 2，机器狗世界位姿没有跳变。停止原因不是切换点错误，
而是切图 Action 发布 `floor_switch_hold=false` 后立即返回成功；任务执行器随即
发送 F2 首个导航目标时，导航网关仍保留上一拍的 `hold=true`，因此拒绝目标并令
任务进入 `FAULT_HOLD`。

导航网关现对与目标 floor/generation 匹配的新目标增加一个有界的解除等待窗口。
若它自己的可靠订阅仍是 `hold=true`，先保持目标不发布并等待本节点收到
`hold=false`；收到后才 reset SCAN 并发布目标。这样同步依据是实际会拒绝目标的
导航网关自身状态，而不是由任务执行器推测另一个 DDS 订阅者已经收到消息。等待
超时或地图代次改变仍然 fail-closed。任务重试时若目标楼层已提交且 hold 已解除，
也会识别为已完成事务，不再重复切图。

高密度 PCD 的 reload 和第一批原生 ray-cast 云耗时存在波动，因此该独立 profile
把 sensing readiness timeout 从 5 s 调整为 12 s；安全 hold 在至少 3 帧目标楼层
新点云到达之前不会解除。

### 原生点云传输补充修复

解除上述 hold 竞态后，回归又暴露出第二个独立问题：renderer 已发布
`/quad_0/cloud`，并且能看到 3 个匹配订阅者，但强制 `lo` 的 CycloneDDS profile
在当前 WSL/Hyper-V 环境会静默丢弃 PointCloud2。原 PCL `PointXYZI` 还带有内存
对齐填充，每点占 32 字节，单帧约 75–78 KiB。

修复分为运行语义和传输配置两层：

1. renderer 用标准 `x/y/z/intensity` 四个 `float32` 字段构造 16 字节 PointCloud2，
   首帧实测约 31–39 KiB，不减少有效点；
2. 仿真 profile 可显式启用可靠 cloud QoS，上游默认仍保持 best-effort；
3. `cyclonedds_local.xml` 只扩展 participant 数量，不再强制 `lo` 或接近 UDP 极限的
   message size；使用独立 `ROS_DOMAIN_ID` 隔离仿真；
4. 高密度 CPU profile 保持 0.5° 极坐标分辨率，并关闭对已按 0.1 m 采样表面的重复
   平面插值，最近射线遮挡逻辑不变。首帧 ray-cast 实测约 0.02 s。

在独立 domain 79 的无 GUI 节点图中，F1 首帧点云到达后，机器狗在共享原点执行
F1→F2，F2 PCD reload 和新 generation 点云均完成，得到
`PHASE5_SWITCH_STRESS_PASS`。切换期间没有 hold 速度违规。

## 返回总起点

原 `terminal` 只允许 `elevator_lobby`。本次把终点位置扩展为两个受控枚举：

```text
elevator_lobby
initial_pose
```

新任务最后使用 F1 的 `initial_pose`，即 `(-37, 0, 0)`。在此之前先执行
`E1:F2->F1`。切换管理器复用已实现的 `reverse_transition_enabled`，在共享原点
原地切换 active map，机器人 odom 不被改写。

## 启动配置参数化

`inspection_mission_rviz.launch.py` 和 `multifloor_scan_rviz.launch.py` 新增
`system_config` 参数。底层节点图保持一份，新入口仅选择新 profile，避免复制并分叉
整套启动逻辑。

日常启动仍是两步：

```bash
ros2 launch m20_warehouse_inspection \
  dense_four_corner_mission_rviz.launch.py

ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id dense_four_corner_patrol
```

## 验收状态

- 配置和生成器单元测试：通过；
- 生成资产校验：通过；
- Python 风格与文档字符串检查：通过；
- 四个项目包静态测试：83 项通过；
- 修改涉及的五个包构建：通过；
- 安装空间配置与资产校验：通过；
- 独立工作空间准备：固定 revision 和两个校验补丁实际复制通过；
- 全新独立工作空间 18 包闭包构建：通过，用时 9 min 45 s；
- F1→F2 切换专项回归：`PHASE5_SWITCH_STRESS_PASS`；
- ROS 完整 11 步节点图：`PHASE4_FULL_ACCEPTANCE_PASS`，最终
  F1/generation=3，切换跳变量 0.000 m，起点返回误差 0.118 m。
