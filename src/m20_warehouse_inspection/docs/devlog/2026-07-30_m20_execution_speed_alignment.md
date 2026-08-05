# M20规划与执行速度对齐

日期：2026-07-30

> 状态：已撤回。当前主流程不再覆盖 SCAN 的规划或闭环速度，保持原项目
> `0.75 m/s`；速度兑现与限幅问题转由下游 M20 运动适配层处理。本文仅保留为
> 试验过程记录。

## 问题

完整系统此前沿用原版 SCAN 的 `0.75 m/s` 轨迹速度和闭环速度，但导航适配器与
M20 SDK 接口将实际前进命令限制为 `0.45 m/s`。SCAN 的局部轨迹时间按墙钟推进，
因此规划状态可能领先于 MuJoCo 中的真实机体。重规划从真实位置开始时仍可能继承
旧轨迹在较晚时刻的导数，造成新旧轨迹切向和机体朝向不一致。

本轮只实施速度链对齐，不修改重规划条件、朝向控制、膨胀半径或 rebound 优化器。

## 官方依据与项目取值

云深处 M20 官方产品页给出的最大工作速度为 `2 m/s`：

<https://deeprobotics.cn/robot/index/lynx.html>

工作区内同步的官方训练配置
`third_party/rl_training/.../deeprobotics_m20/rough_env_cfg.py` 使用：

```text
lin_vel_x = [-2.0, 2.0] m/s
lin_vel_y = [-1.0, 1.0] m/s
ang_vel_z = [-1.0, 1.0] rad/s
```

官方部署策略在 `M20PolicyRunner::getRobotAction()` 中把
`forward_vel_scale / side_vel_scale / turnning_vel_scale` 直接写入策略观测，
没有把 `0.40 m/s` 声明为硬件极限。

因此本项目明确区分：

- `2.0 m/s`：官方最大工作速度和策略前进训练边界；
- `0.40 m/s`：双层仓库、0.90 m 最小障碍间距下采用的保守规划/巡检速度；
- `0.45 m/s`：SDK 接口保留的下游保护上限。

0.40 m/s 是官方工作上限的 20%，用于给窄通道跟踪、转向和停止过程留出余量。

## 实施

新增 `m20_scan_navigation/config/scan_m20_physical_planner.yaml`：

```yaml
manager.max_vel: 0.40
optimization.max_vel: 0.40
```

在 `scan_m20_physical_controller.yaml` 中增加：

```yaml
max_vx: 0.40
```

参数加载顺序为：

```text
SCAN vendor planner (0.75 m/s)
  -> M20 physical planner override (0.40 m/s)
  -> clearance profile (只覆盖净空参数)
```

完整 RViz、双层巡检和 MuJoCo 启动默认使用 M20 覆盖配置。直接启动
`m20_scan_navigation/f1_scan.launch.py` 时仍保持原版 SCAN 参数，便于算法对照。

## 预期效果

- SCAN 轨迹参数化速度不再超过实际可执行速度；
- 正常命令不再由 SDK 从 0.75 m/s 二次截断到 0.45 m/s；
- 以规划轨迹时间触发的 1 m 滚动重规划与真实运动进度更接近；
- 原版 SCAN 独立对照入口、膨胀参数和可视化保持不变。
