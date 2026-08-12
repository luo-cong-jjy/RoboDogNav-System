# 障碍物最小间距对比场景开发记录

日期：2026-07-28

## 目标

在不覆盖现有 `route_challenge` 的前提下，生成四套可直接启动的双场景测试资产，
分别强制障碍物本体最小间距不小于 `0.70 / 0.80 / 0.90 / 1.00 m`，用于观察
M20双圆柱模型、SCAN局部规划和独立 `0.30 m` 碰撞保护在狭窄环境中的表现。

## 控制变量

四个档位保持以下内容一致：

- 每层252个障碍物，其中240个确定性随机障碍物和12个固定绕障障碍物；
- F1/F2场景尺寸、共享原点门洞、巡检点和11步任务顺序；
- 随机种子127；
- 原生SCAN规划器、控制器、M20模型和安全参数；
- 障碍物尺寸采样范围。

原挑战图中的 `inbound_left_pallet` 与 `left_descent_case` 只有 `0.80 m` 间距。
为了让固定布局同时满足最高 `1.00 m` 档位，四套对比图统一把不截断任务路线的
`inbound_left_pallet` 中心移动到 `(-38.35, -7.7)`。这个调整不写回原挑战图。

## 实现

- `tools/generate_spacing_sweep.py`
  - 从 `route_challenge_system.yaml` 派生四份完整配置；
  - 对配置和固定障碍物先做系统级校验；
  - 确定性生成PCD、PGM、地图YAML和JSON元数据；
  - 重新读取资产并校验哈希、数量、复制关系和最小间距。
- `launch/spacing_sweep_mission_rviz.launch.py`
  - 通过单一 `spacing` 参数选择四个档位；
  - 继续复用正式 `inspection_mission_rviz.launch.py` 节点图；
  - 默认保持 `use_grid_route:=false`，直接使用原生SCAN目标链。
- `test/test_spacing_sweep.py`
  - 检查四套配置只改变实验档位；
  - 检查252个障碍物和实际本体间距；
  - 按独立碰撞保护的 `0.30 m` 半径膨胀占据图；
  - 检查F1六段、F2五段共11段任务端点连通。

## 生成结果

| 配置 | 请求间距 | 实际最小间距 | 每层障碍物 |
|---|---:|---:|---:|
| `spacing_sweep_070_system.yaml` | `0.70 m` | `0.702172 m` | 252 |
| `spacing_sweep_080_system.yaml` | `0.80 m` | `0.805815 m` | 252 |
| `spacing_sweep_090_system.yaml` | `0.90 m` | `0.900236 m` | 252 |
| `spacing_sweep_100_system.yaml` | `1.00 m` | `1.001421 m` | 252 |

较大间距会改变确定性拒绝采样的接受序列，因此四套随机障碍物并非逐个保持原坐标。
它们保持相同随机种子、数量和生成规则，适合做单因素场景系列，但单次耗时不能被
解释成严格单调的性能曲线。

## 使用方法

第一终端：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash

ros2 launch m20_warehouse_inspection \
  spacing_sweep_mission_rviz.launch.py spacing:=0.90
```

第二终端：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash

ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id spacing_sweep_four_corner_patrol
```

切换档位前先正常结束上一套launch，再把 `spacing` 替换为另一个合法值。
