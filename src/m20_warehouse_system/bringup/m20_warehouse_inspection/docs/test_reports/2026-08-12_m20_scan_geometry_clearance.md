# M20 原生 SCAN 安全几何回归报告

日期：2026-08-12

## 对照参数

| 配置 | 硬半径 | 前后圆心偏置 | 软距离 | 硬轴向外廓 | 名义直通道 |
|---|---:|---:|---:|---:|---:|
| 上游SCAN/Go2 | 0.25 m | 0.18 m | 0.20 m | 0.86 × 0.50 m | 0.90 m |
| 完整M20默认 | 0.28 m | 0.18 m | 0.17 m | 0.92 × 0.56 m | 0.90 m |

M20确认外廓为`0.82 × 0.51 × 0.57 m`。新参数把总长宽轴向余量增至约
`0.10/0.05 m`，且不修改A*、rebound、B-spline、闭环控制和可视化源码。

双圆并非矩形四角的严格外接。严格双圆近似至少需约`0.327 m`半径；已有否决实验
确认`0.32/0.35 m`在`0.05 m`栅格上会封闭`0.90 m`门洞。因此本配置是经过通行性
约束的安全增量，真机仍需低速验收。

## 自动测试

```text
几何/导航/集成定向回归：47 passed
最终五包逐包源测试：225 passed
m20_scan_navigation + m20_warehouse_inspection：build passed
```

`scan_native`与`m20_safe`的保护门控已分开：前者不等待未启动的保护节点，后者继续
失效关闭。窄通道启动入口默认值同步为`clearance_profile:=m20`。

## 运行验证

### RViz运动学

- 起点：`(-37.000, 0.000)`；目标：`(-35.000, 0.000)`；
- 最终：`(-35.031, 0.000)`，误差约`0.031 m`；
- 最终状态`WAIT_TARGET`，未出现A-star或robot-inside-obstacle错误。

### MuJoCo/官方SDK动力学

- 场景：`0.90 m`标准门洞；
- 起点：`(-18.000, 0.000)`；目标：`(-7.000, 0.000)`；
- 最终：`(-7.021, -0.046)`，二维误差约`0.050 m`；
- `obstacle_contact_count=0`；
- `obstacle_contact_event_count=0`；
- `obstacle_contact_peak_force_n=0.0`；
- `backend_fault=""`，停止后驻车制动生效。

该次使用完整一键MuJoCo/SDK图和默认M20参数，但以直接SCAN目标完成运动，用于隔离并
验证几何与动力学。随后正式Action冷启动复验第一次受宿主机遗留的第二套MuJoCo/SDK
进程干扰而跌倒；清理后再次启动被当前执行环境的授权审查中断，故Action项明确保留
为待补跑，不虚报通过。

## 复验命令

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash

ros2 launch m20_warehouse_inspection \
  narrow_passage_mission_mujoco.launch.py \
  width:=0.90 clearance_profile:=m20
```

另开终端：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash
ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id narrow_passage_benchmark
```
