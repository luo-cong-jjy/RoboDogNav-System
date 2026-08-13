# MuJoCo自由导航运动基线报告

日期：2026-07-29

## 汇总

| 目标 | 结果 | 耗时 | 终点误差 | 最大横向偏离 | 轮腿同时参与 |
|---|---:|---:|---:|---:|---:|
| 2.5 m直线前进 | PASS | 11.59 s | 0.152 m | 0.045 m | 57.1% |
| 2.5 m 180°回程 | PASS | 22.99 s | 0.159 m | 0.611 m | 100.0% |
| 右转90°后前进6 m | PASS | 38.06 s | 0.192 m | 0.099 m | 38.0% |
| 左转90°后前进3 m | PASS | 27.05 s | 0.157 m | 0.094 m | 99.6% |
| 3 m 180°回程 | PASS | 27.27 s | 0.159 m | 0.564 m | 94.5% |
| 左转90°返回起点6 m | PASS | 48.63 s | 0.153 m | 0.228 m | 97.6% |

总计6/6目标完成，总耗时175.59 s，共收到72条SCAN B样条轨迹，没有
`collision_stop`样本。

## 关键结论

- 首段直线轨迹几何偏离小，但闭环偏航命令持续存在；
- 官方策略在纯`vx`、零`vy/wz`输入下仍同时驱动腿和轮；
- 全部运动采样中轮腿同时明显参与比例为81.76%；
- `LATERAL_MANEUVER`占全部运动采样63.81%，是明显腿部动作和低速前进的重要来源；
- 180°动作扫掠不能按零半径原地旋转处理。

## 数据

- 原始采样：
  `artifacts/navigation_motion/2026-07-29_baseline/navigation_motion_samples.csv`
- 汇总指标：
  `artifacts/navigation_motion/2026-07-29_baseline/navigation_motion_summary.json`
- 可复现实验工具：
  `src/m20_warehouse_inspection/tools/navigation_motion_probe.py`

本报告是当前参数基线，不是后续适配方案的通过报告。下一阶段需要在独立配置中
实现滚动导航适配候选，并重复相同六目标做A/B验收。
