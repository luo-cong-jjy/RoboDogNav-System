# 原版 SCAN 与 M20 执行链对照测试报告

日期：2026-07-31

## 结论

本轮已实际运行用户指定的
`src/third_party/SCAN-Planner`
场景 1 和当前 M20 官方 SDK + MuJoCo 主线。

原版场景不是更宽松：500 个障碍没有最小间距约束，实体最小正间隙仅
0.003385 m；当前场景 246 个障碍的实体最小间隙为 0.900236 m。原版导航灵活的
直接原因是它使用理想全向平面积分器，能够无误差执行 SCAN 的 `vx/vy/wz`，
而不是 RViz 中显示的 Go2 在执行四足动力学。

同路线动态实验中，原版 6/6 成功；当前 M20 在第一个无障碍直线目标上因
`EXCESSIVE_TILT` 失败。故障时独立 guard 余量仍为 1.174 m，且碰撞停止为 0，
因此该次失败与实体障碍或膨胀区无关。当前首要修复范围应是 M20 运动适配和
SDK 动态稳定包线，不应再改变 SCAN 规划参数。

## 静态场景结果

| 指标 | 原版 SCAN | 当前 M20 场景 |
|---|---:|---:|
| 障碍数量 | 500 | 246 |
| 障碍原语面积和 | 142.114675 m² | 103.693154 m² |
| 重叠障碍对 | 99 | 0 |
| 最小正间隙 | 0.003385 m | 0.900236 m |
| 正间隙 `< 0.10 m` 的障碍对 | 29 | 0 |
| 正间隙 `< 0.50 m` 的障碍对 | 185 | 0 |
| 正间隙 `< 0.90 m` 的障碍对 | 429 | 0 |

原版统计由独立 C++ 工具严格复现 Mockamap 的
`std::default_random_engine(127)`；当前值来自冻结的 scene JSON 和最小间距
校验逻辑。

## 模型与执行器差异

| 项目 | 原版场景 1 | 当前 M20 |
|---|---|---|
| RViz 外观 | Unitree Go2 URDF | Deep Robotics 官方 M20 URDF |
| 导航执行模型 | 理想全向平面数值积分 | 官方 ONNX + 16 执行器 + MuJoCo |
| 横移 | `vy` 直接积分 | 由轮足策略协调腿和轮 |
| 轮地/足地接触 | 无 | 有 |
| 质量/惯量 | 导航执行时忽略 | 使用官方 MJCF |
| 输入到实测速度误差 | 近似 0 | 需要动态闭环控制 |
| 倾覆约束 | 无 | 有，当前 1.20 rad 后端故障阈值 |

Go2 URDF 的视觉/关节动画不参与导航位姿积分，因此不能把原版结果解释为
“Go2 四足模型比 M20 更灵活”。

## 动态结果

### 原版 SCAN

| 指标 | 结果 |
|---|---:|
| 目标成功率 | 6/6 |
| 总用时 | 71.46 s |
| 最终误差中位数 | 0.00366 m |
| 原始横移命令样本占比均值 | 53.34% |
| raw → applied 线速度 RMS 差 | 0 |
| applied → realized 线速度 RMS 差均值 | 0.00637 m/s |
| B-spline 消息总数 | 36 |

### 当前 M20

| 指标 | 结果 |
|---|---:|
| 第一个 2.5 m 目标 | 失败 |
| 故障前行驶距离 | 1.627 m |
| 最大横向偏差 | 0.362 m |
| raw 横移命令样本占比 | 28.09% |
| applied 横移命令样本占比 | 0% |
| raw → applied 线速度 RMS 差 | 0.310 m/s |
| applied → realized 线速度 RMS 差 | 0.423 m/s |
| 最小 SCAN 硬边界余量 | 1.224 m |
| 最小 guard 余量 | 1.174 m |
| collision stop | 0 |
| MuJoCo fault | `EXCESSIVE_TILT` |

故障前适配器将原始
`(vx≈0.55, vy=-0.35, wz≈-0.96)`
转换为
`(vx=0.05, vy=0, wz=-0.65)`。
这说明当前“删除横移并把横移误差叠加到偏航”的转换与原闭环对象差异很大。

## 已新增的可复现资产

| 资产 | 用途 |
|---|---|
| `tools/scan_vendor_scene1_spacing.cpp` | 精确复现原版场景 1 障碍统计 |
| `tools/scan_execution_comparison_probe.py` | 同路线记录两套执行链 |
| `docs/test_data/2026-07-31_scan_motion_comparison/` | 原始 CSV 和 JSON |

记录器现已订阅 MuJoCo 后端 fault；后续一旦故障会立即结束当前目标并落盘，不再
等待导航超时。运动意图订阅也已改为与实际 volatile publisher 兼容的 QoS。

## 工程验证

| 验证项 | 结果 |
|---|---:|
| `m20_warehouse_inspection` Python 回归 | 70 passed |
| `m20_locomotion_control` 运动意图回归 | 16 passed |
| 对照记录器 `flake8 / pydocstyle / py_compile` | passed |
| 场景统计器 `cpplint / uncrustify / cppcheck` | passed |
| 场景统计器复编译、复运行 | passed |
| `m20_warehouse_inspection` 重新构建 | passed |
| 主 MuJoCo launch `--show-args` | passed |
| `git diff --check` | passed |

## 放行判断

当前结果不支持继续通过减小 SCAN 膨胀层解决问题。下一步应先完成：

1. 官方策略 `vx/vy/wz` 联合稳定包线测试；
2. 去除 `raw_wz + 1.20 * lateral_course_error` 的重复纠偏；
3. 以联合曲率、加速度和实测速度反馈生成 M20 可执行命令；
4. 依次复验直线、空旷转向、单障碍、0.90 m 窄道和六目标。

详细原理、命令和分阶段实现方案见同日开发记录。
