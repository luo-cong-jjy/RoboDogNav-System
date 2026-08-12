# SCAN 贴边路径净空代价测试报告

日期：2026-07-31

> 状态：对应实现已撤回，以下结果不代表当前默认系统。

## 正式参数

```yaml
grid_map.double_cylinder_radius: 0.30
optimization.dist0: 0.15
path_searching.clearance_margin: 0.15
path_searching.clearance_weight: 2.0
optimization.global_clearance_weight: 2.0
```

硬占据范围没有缩小或增大，新增部分只在 `0.30～0.45 m` 配置空间内改变自由
路径的相对代价。

## 结果

| 验证项 | 结果 |
|---|---:|
| 导航与集成 Python 回归 | 92 passed |
| A* 净空代价 C++ | 4 passed |
| B-spline C++ | 2 passed |
| 相关依赖链构建 | 6 packages passed |
| git diff 格式检查 | passed |

A* 单元测试覆盖：

- 权重为零时保持 vendor 基础代价；
- 代价从硬层边界向软带外缘单调下降；
- 软带外缘代价严格为零；
- 当前权重下，合理的开阔绕行优先于更短的贴边路径。

## 待完成

正常 DDS 环境中的验收需要同时满足：

1. `0.90 m` 受控通道任务完成；
2. `/a_star_list` 在有额外空间时不再沿膨胀层第一格走；
3. `/planning/bspline` 不因平滑重新切回软带内侧；
4. 无 `CURRENT_FOOTPRINT`、无 MuJoCo 实体障碍接触；
5. 完整高密度双场景任务不出现新的 A* 0.2 s 超时。
