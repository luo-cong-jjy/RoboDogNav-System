# 高密度四角巡检副本测试报告

日期：2026-07-28

## 结果摘要

| 项目 | 结果 |
|---|---|
| 新旧系统配置 schema | PASS |
| 两场景确定性地图生成 | PASS |
| 每场景 246 个障碍物（240 随机 + 6 固定） | PASS |
| 最小边界距离 ≥ 0.7 m | PASS，实测 0.702172 m |
| 固定障碍物配置、元数据及副本继承 | PASS |
| F2 内部布局复制 F1 | PASS |
| 四角点顺序 | PASS |
| F2→F1 反向切层解析 | PASS |
| 最终终点等于 F1 初始位姿 | PASS |
| 旧地图 PCD/PGM 哈希不变 | PASS |
| Python 单元/集成静态测试 | PASS，83 项 |
| ament_flake8 | PASS，47 个文件 |
| ament_pep257 | PASS |
| 修改涉及的五个包构建 | PASS |
| 安装空间配置及资产校验 | PASS |
| 独立工作空间准备及 18 包闭包构建 | PASS，9 min 45 s |
| F1→F2 切换专项回归 | PASS |
| ROS 11 步完整节点图运行 | PASS |

## 执行命令

```bash
PYTHONPATH=src/m20_warehouse_inspection:src/m20_inspection_core:\
src/m20_scan_navigation:src/m20_multifloor_map \
  /usr/bin/python3 -m pytest -q \
  src/m20_warehouse_inspection/test \
  src/m20_inspection_core/test \
  src/m20_scan_navigation/test \
  src/m20_multifloor_map/test
```

结果：

```text
83 passed in 2.46s
```

```bash
/opt/ros/humble/bin/ament_flake8 \
  src/m20_warehouse_inspection/launch \
  src/m20_warehouse_inspection/m20_warehouse_inspection \
  src/m20_warehouse_inspection/test \
  src/m20_inspection_core/m20_inspection_core \
  src/m20_inspection_core/test
```

结果：

```text
47 files checked
No problems found
```

```bash
colcon build --symlink-install --packages-select \
  local_sensing_node m20_multifloor_map m20_scan_navigation \
  m20_inspection_core m20_warehouse_inspection
```

结果：

```text
Summary: 5 packages finished
```

安装空间校验结果：

```text
configuration valid: 2 floors, layout=flat_connected_regions
F1: 186693 points, 246 obstacles,
sha256=2feed3a640bf81a8c11a74f397ee8120a355c85994ecfa4f638d818e90b15876
F2: 186693 points, 246 obstacles,
sha256=c1b7817061ed2ed45ddfee9a88ced1c068e2a10e86aa9ddf6e2e341a9c70c6ea
```

## 固定障碍物运行试验

第一次把 1 m × 1 m 固定障碍物放在巡检中心线上。机器狗接近下侧第一个障碍物时，
独立碰撞保护正确进入安全停车，但原生 SCAN 没有稳定绕过该正面封堵。该方案被撤回。

第二次把六个固定障碍物交错移到巡检过道边缘，固定障碍物边缘距离巡检中心线
0.8 m。完整运行通过 F1/F2 全部四角巡检腿。F2 切换后的首次出发出现一次
0.150 s 的短时 `COLLISION_STOP` 并自动恢复，任务未进入故障；其余固定障碍路段
没有持续安全停车。

## 场景切换故障复现与修复

故障运行中第五步已出现 `active map committed: F2, generation=2`，随后第六步目标
因为 `floor switch hold is active` 被拒绝，任务进入 `FAULT_HOLD`。这证明机器狗
停在出口的直接原因是 hold 解除消息的消费竞态，而不是共享切换点坐标错误。

修复后由导航网关在执行新 floor/generation 目标前等待它自己的可靠 hold 解除样本，
不再由任务执行器推测另一个订阅者的状态；高密度 profile 的点云就绪窗口同时从
5 s 调整为 12 s，并继续要求至少 3 帧目标楼层新点云。完整 11 步结果将在本次运行
结束后补入下节。

随后还复现了“发布器已匹配 3 个订阅者但原生点云计数为 0”。对 renderer 分阶段
计时和消息字节数检查后完成以下修复：

- PointXYZI ROS 消息从带 PCL 对齐填充的 32 字节/点改为标准 16 字节/点；
- 仿真 profile 启用可选 reliable cloud QoS；
- DDS 配置不再强制当前 WSL 环境会丢 PointCloud2 的 `lo`，仅扩展 participant；
- 使用独立 `ROS_DOMAIN_ID=79` 隔离回归。

切换专项回归实测 F1 首帧 2051 点/32816 字节、F2 首帧 1940 点/31040 字节，
F1→F2 generation 从 1 变为 2，结果为：

```text
PHASE5_SWITCH_STRESS_PASS: 1 alternating switches,
no hold velocity violations
```

## 完整运行结果

无 RViz 的 `acceptance_mode:=full` 完成全部 11 步：

```text
PHASE4_FULL_ACCEPTANCE_PASS: 11 steps,
mission=dense_four_corner_patrol,
pause/resume,
zero-jump origin map switch,
final_floor=F1,
generation=3,
switch_jump=0.000m,
final_error=0.118m
```

- F1→F2 在共享原点完成，F2 active map 为 generation 2；
- 第六步正常进入 F2 左下巡检，没有再次进入 `FAULT_HOLD`；
- F1/F2 八个四角点全部完成；
- F2→F1 仍在共享原点完成，F1 active map 为 generation 3；
- 最终返回 `(-37, 0)`，0.118 m 误差小于 0.20 m 导航容差；
- 两次地图切换均没有改写机器人世界位姿。
