# 正式双场景障碍物间距调整至0.90 m开发记录

日期：2026-07-29

## 目标

将完整RViz/MuJoCo系统默认使用的`dense_four_corner`双场景从0.70 m
障碍物本体边界最小间距调整为0.90 m，同时保持：

- 每层246个障碍物不减少；
- F2继续严格复制F1内部障碍物布局；
- 两层相邻、共享原点门洞和无瞬移切图语义不变；
- 四角巡检顺序、11步任务和最终返回全流程起点不变；
- 0.70/0.80/0.90/1.00 m间距实验资产继续独立保留。

这里的0.90 m定义为任意两个独立矩形障碍物本体边界之间的二维欧氏距离，
不是规划器到单个障碍物的净空，也不能单独替代窄通道动态验收。

## 配置与资产修改

正式配置`config/dense_four_corner_system.yaml`中的：

```yaml
map_generation:
  minimum_obstacle_spacing: 0.90
```

生成器继续使用固定种子127和原有保护区，在不降低障碍物数量的前提下重新放置
240个随机障碍物；6个固定过道障碍物保持不变。随后重新生成两层PCD、PGM、
地图YAML和JSON元数据。MuJoCo碰撞世界在启动时读取同一JSON，因此不维护第二套
手工几何文件。

生成命令：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/m20_warehouse_inspection \
python3 src/m20_warehouse_inspection/scripts/m20_generate_maps \
  --config \
  src/m20_warehouse_inspection/config/dense_four_corner_system.yaml \
  --package-root src/m20_warehouse_inspection
```

## 生成结果

| 场景 | 障碍物 | 点数 | 配置阈值 | 实际最近边界距离 |
|---|---:|---:|---:|---:|
| F1 | 246 | 185392 | 0.90 m | 0.900236 m |
| F2 | 246 | 185392 | 0.90 m | 0.900236 m |

新冻结哈希：

| 文件 | SHA-256 |
|---|---|
| `dense_four_corner_system.yaml` | `5f1d52a1a5e510d71bd18b26e4b05d2b3d5ea11618b82da3729b078689e75738` |
| F1 PCD | `03ca6a784b712ecacb644bafaec2293acb3b7ba18a36b559d7de089ff910cc20` |
| F1 PGM | `fb16e7b0b9f485da54c975b3825a8d4893e5b6eee7da4fbe5cc350132602ac87` |
| F2 PCD | `cf9b271878b6c8e9c90f6e342dfc408cbb827b573a6ad24c2227e3e71a16b720` |
| F2 PGM | `fd3e996dca065d6bb4e5b788013556970a6b842e19ca466ce5c22743907870a1` |

`config/rviz_v1_baseline.yaml`已同步上述哈希和本次重冻结原因，防止已安装地图或
后续手工修改与正式默认配置静默不一致。

## 回归约束

地图测试除读取JSON中的0.90 m声明外，还逐对计算全部246个矩形的边界距离。
随后将0.10 m占据图按独立碰撞保护半径0.30 m膨胀，验证以下锚点仍在同一自由空间
连通区域：

- F1起点、F1四角巡检点、F1共享门洞；
- F2共享门洞、F2四角巡检点。

这项静态连通性回归可防止提高间距后随机布局变化意外封闭任务区域，但不代表
SCAN轨迹跟踪和MuJoCo轮腿动力学已经完成全任务动态复验。

## 动态验收边界

此前11/11完整MuJoCo任务使用的是历史0.70 m默认资产。当前0.90 m资产已经完成
生成、配置、连通和包级回归，但尚未重新执行一次完整11步MuJoCo任务。因此：

- 可以作为新的正式默认场景启动；
- 不能把历史0.70 m的耗时、最小净空和成功率直接记到0.90 m结果中；
- 后续应使用当前默认启动入口完成至少一次无重试11步验收，再进入多轮稳定性统计。
