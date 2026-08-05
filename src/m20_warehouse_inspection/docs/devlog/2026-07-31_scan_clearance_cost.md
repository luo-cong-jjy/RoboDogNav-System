# 2026-07-31 SCAN 贴边路径净空代价改进

> 状态：已撤回。当前代码不包含 `getInflateClearance`、A* 净空边代价或
> B-spline 全局净空项；主流程已恢复第三方 SCAN 的二值 A* 和原优化目标。
> 本文仅保留为试验过程记录。

## 问题

`grid_map.double_cylinder_radius` 确实参与原生 rebound A* 的碰撞查询，但原算法
只区分占据和自由。硬膨胀层外第一个自由栅格与开阔区域的基础代价相同，因此最短
路径可以合法地贴着膨胀边界。

继续减小硬半径只会让路径更靠近实体障碍；继续增大硬半径又会封闭 `0.90 m`
通道。`optimization.dist0` 原本只对已识别的 rebound 碰撞段建立方向，不能覆盖
一条没有穿入硬层、但已经进入安全余量的重规划轨迹。

## 设计

保留两个不同含义的区域：

```text
实体障碍至 0.30 m：硬占据，绝对禁止
0.30 m 至 0.45 m：自由但有净空代价
大于 0.45 m：普通自由空间
```

新增 `GridMap::getInflateClearance()`。查询仍使用带航向的前后双圆模型，通过在
配置空间采样机器人姿态，返回当前自由姿态到最近硬占据姿态的距离和远离方向。

### A* 代价

当净空 `d` 小于软带宽度 `m` 时：

```text
penalty(d) = weight * (1 - d / m)^2
edge_cost = geometric_step * (1 + penalty)
```

正式值：

```yaml
path_searching.clearance_margin: 0.15
path_searching.clearance_weight: 2.0
```

这是正的附加代价，不改变任何栅格的占据状态。启发函数仍忽略附加代价，因此不会
把可行窄通道误判成不可达。

### B-spline 代价

对每个可优化控制点查询同一净空。若控制点进入软带，则增加：

```text
cost = weight * (m - d)^3
```

梯度沿 `getInflateClearance()` 返回的远离障碍方向。正式值：

```yaml
optimization.global_clearance_weight: 2.0
```

原 rebound 碰撞段代价保留；新增项补足无碰撞但贴边的控制点。

## 兼容性

- `scan_vendor_planner.yaml` 没有新增覆盖；
- 新参数在代码中的默认值均为 `0.0`；
- 单独启动 `m20_scan_navigation/f1_scan.launch.py` 时 A* 和优化器行为不变；
- 完整 M20 系统通过 `clearance_conservative.yaml` 启用；
- 点云、占据层、A*、B-spline 和 RViz 原话题与显示接口均未改变。

## 修改范围

- `plan_env`：配置空间净空与梯度查询；
- `path_searching`：自由栅格净空边代价；
- `bspline_opt`：所有控制点的软带距离代价；
- 两份 planner manager：读取参数并传递给 A*；
- conservative 配置、契约测试和说明文档。

## 验证

- 六个相关包完整重新构建成功；
- 导航与集成 Python 回归：`92 passed`；
- A* 净空代价 C++：`4 passed`；
- B-spline C++：`2 passed`；
- 参数级 `0.90 m` 通道仍为 `NOMINAL_CANDIDATE`；
- 新增测试确认当前权重下，12 格开阔绕行的总代价低于10格贴边路径。

当前自动执行环境不能创建 DDS socket，因此本轮不宣称新的 MuJoCo 动态通过。
需要在正常本机环境观察 `/a_star_list` 与 `/planning/bspline` 相对
`/grid_map/occupancy_inflate` 的净空，并复验完整高密度任务。
