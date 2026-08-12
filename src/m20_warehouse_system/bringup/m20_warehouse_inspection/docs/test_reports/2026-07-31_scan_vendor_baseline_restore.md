# SCAN 原项目基线恢复测试报告

日期：2026-07-31

## 结论

本轮已撤回会改变 SCAN 路径选择或闭环控制行为的 M20 专用试验。完整 RViz、
自动巡检和 MuJoCo 主入口重新使用固定第三方 `SCAN-Planner` 的规划与控制参数。
当前差异仅保留系统集成所需接口，不再由 SCAN 内部补偿 M20 运动学。

## 参数一致性

逐键比较 `scan_vendor_planner.yaml` 与固定第三方 `planner.yaml`，所有上游参数
一致；仅保留任务 Action 所需：

```yaml
fsm.target_reached_tolerance: 0.20
```

逐键比较 `scan_vendor_controller.yaml` 与固定第三方 `controllers.yaml` 中的
闭环控制器，所有上游参数一致；仅保留暂停/切层/碰撞保持所需：

```yaml
execution_hold_topic: /m20/control/execution_hold
require_external_execution_hold: true
```

安装后的主入口解析结果为：

```text
clearance_config = clearance_vendor.yaml
controller_config = scan_vendor_controller.yaml
```

安装后的核心值为：

```text
grid_map.double_cylinder_radius = 0.25
optimization.dist0 = 0.20
manager.max_vel = 0.75
optimization.max_vel = 0.75
heading_error_threshold = 0.80
max_vx / max_vy / max_vyaw = 0.75 / 0.35 / 1.00
```

## 静态和构建验证

| 验证项 | 结果 |
|---|---:|
| `plan_env`、`path_searching`、`bspline_opt` 及三个 M20 集成包构建 | 6 passed |
| 导航与仓库系统 Python 回归 | 92 passed |
| 主 MuJoCo launch `--show-args` | passed |
| vendor planner/controller 参数逐键一致性 | passed |
| SCAN 控制器额外航向迟滞/平移缩放符号检查 | passed |
| clearance A*/B-spline 试验符号残留检查 | none |
| `git diff --check` | passed |

构建 stderr 仅包含 `plan_env` 和 `bspline_opt` 固定上游代码已有的编译警告，
没有接口缺失、链接失败或参数声明错误。

## 本轮未宣称的结果

当前执行环境不允许建立 DDS 共享内存/网络通信，因此本报告不把静态回归等同于
MuJoCo 动态放行。下一轮用户侧复验应保持 SCAN 基线不动，记录：

1. `/m20/navigation/cmd_vel_raw` 与 `/m20/control/cmd_vel_safe`；
2. 规划切线方向、M20 实际机身朝向与横向速度兑现误差；
3. 首次进入膨胀占据前后的 adapter 状态和 collision guard 状态；
4. 同一目标在原版平面执行后端与 MuJoCo/SDK 执行后端的差异。

若只有 MuJoCo/SDK 链路出现偏航、贴边或停滞，后续修复范围限定在
`m20_navigation_adapter`、反馈闭环和官方 SDK 接口，不再修改 SCAN 规划代价。
