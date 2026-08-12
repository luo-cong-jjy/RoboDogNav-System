# 2026-08-10 集成闭包全新构建与自由导航冒烟测试

## 测试目的

验证默认 colcon 发现范围已收敛到锁定的生产闭包，并确认清理旧 build/install/log 后，
`m20_warehouse_inspection` 单一入口仍能完成 RViz 平面后端的原生 SCAN 自由导航。

## 构建结果

- `colcon list`：22 个包，与 `workspace_lock.yaml.build_closure` 完全一致。
- 全新构建命令：

  ```bash
  colcon build --symlink-install \
    --packages-up-to m20_warehouse_inspection \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  ```

- 结果：22/22 包构建成功。
- 项目包测试：503 tests，0 errors，0 failures，1 skipped。
- SCAN 上游代码存在既有编译 warning，但没有编译或链接错误。

## 导航条件

- 启动：`f1_scan_rviz.launch.py use_rviz:=false`。
- 后端：`m20_warehouse_sim` RViz 平面运动学。
- 导航链：`/move_base_simple/goal -> SCAN -> B-spline -> closed-loop -> safety`。
- 初始位姿：`(-37.0, 0.0, 0.0)`。
- 目标：`(-34.5, 0.0)`，直线距离 2.5 m。
- 超时：60 s；到点连续停稳：1.0 s。

## 结果

| 指标 | 结果 |
| --- | ---: |
| 成功 | true |
| 用时 | 6.98 s |
| 最终位姿 | `(-34.6498, 0.0, 0.0)` |
| 最终位置误差 | 0.150 m |
| 横向最大误差 | 0.000 m |
| B-spline 消息 | 3 |
| collision stop | 0 |
| recovery budget exhausted | 0 |
| backend fault | 0 |
| 启动日志 ERROR/Traceback | 0 |

测试直接向 RViz 点击目标所使用的 `/move_base_simple/goal` 发布 PoseStamped，因此同时
覆盖“点击导航点无反应”故障对应的入口链路。测试后所有 ROS 进程均已退出，原始临时
日志已删除，仅保留本结论记录。

