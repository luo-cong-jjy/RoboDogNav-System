# 2026-08-12 M20 与原生 SCAN 安全几何适配

## 目标与边界

保留已恢复的原生 SCAN 搜索、rebound A*、B-spline、闭环控制和可视化，只调整由
Go2 替换为 M20 后必然不同的机体几何参数。本轮不增加新的路径代价、不恢复执行
guard/hold 链，也不通过改变控制算法掩盖跟踪误差。

## 参数语义审计

- `grid_map.double_cylinder_radius`：占据点的水平硬膨胀半径，也是 A*、轨迹碰撞检查
  和“robot inside obstacle”判定的硬几何边界。
- `grid_map.double_cylinder_offset`：沿轨迹航向放置前后两个圆心，决定轴向长度。
- `optimization.dist0`：只对已识别 rebound 碰撞段施加的软净空代价，不是全局
  距离场，也不能替代机体硬尺寸。
- `grid_map.resolution=0.05 m`：硬半径最终离散为体素模板，小幅参数变化不一定对应
  连续空间的同等变化。
- `grid_map.body_height=0.40 m`：在当前RViz/Action入口中不改变目标高度，目标Z由
  首帧`body_pose`锁定；它只用于未启用的`initial_path`支撑面路径。
- `obstacles_inflation_z_up/down=0.10/0.40 m`：原生三维栅格的上下膨胀，继续处理
  当前平面仓库的墙体点云。本轮不把机身总高直接写入该参数，以免在未做悬空障碍
  数据集验证前改变原项目的三维可通行语义。

上游配置 `radius=0.25 m, offset=0.18 m` 的长宽轴向外廓约为：

```text
length = 2 * (0.18 + 0.25) = 0.86 m
width  = 2 * 0.25 = 0.50 m
```

M20 已确认长宽为 `0.82 × 0.51 m`。上游轴向长度足够，但宽度比 M20 小约
`0.01 m`，且没有栅格、点云和跟踪余量。

## 最小必要适配

完整 M20 系统新增独立 `clearance_m20.yaml`：

```yaml
grid_map.double_cylinder_radius: 0.28
grid_map.double_cylinder_offset: 0.18
optimization.dist0: 0.17
```

结果为：

```text
硬轴向外廓：0.92 × 0.56 m
相对 M20 总余量：长度 0.10 m，宽度 0.05 m
名义单侧包络：0.28 + 0.17 = 0.45 m（不变）
名义直通道：2 * 0.45 = 0.90 m（不变）
```

`m20_safe` 的独立保护器使用 `0.28 + 0.02 = 0.30 m`，保持此前验证过的有效保护
半径；默认 `scan_native` 不启动该保护器。`clearance_vendor.yaml` 保留精确上游
`0.25/0.18/0.20 m`，用于 A/B 对照。

## 为什么不采用严格矩形外接

双圆是方向相关的圆角胶囊近似，`0.92 × 0.56 m` 表示长宽轴向范围，不等于严格
包含长方体四角。对 `0.82 × 0.51 m` 矩形优化两个圆心后，完整包含四角所需半径
至少约：

```text
sqrt((0.82 / 4)^2 + (0.51 / 2)^2) ≈ 0.327 m
```

在 0.05 m GridMap 中这会接近此前试验的 0.35 m 离散膨胀；已有记录显示
`0.32/0.35 m` 会封闭 0.90 m 基准通道。因此选择 0.28 m 是“增加 M20 硬安全几何、
同时保留 0.90 m 通行能力”的工程折中，不是绝对碰撞保证。最终放行还必须满足
MuJoCo 实体接触为零，并通过真机低速净空验收。

## 执行配置门控一致性

完整系统默认`execution_profile=scan_native`时不会启动独立`collision_guard`。动态
Action复验暴露出导航网关仍无条件等待其`CLEAR`，会让自动巡检在发目标前超时，
而手工RViz目标不受影响。现新增显式`collision_guard_required`契约：

- `scan_native=false`：跳过未启动保护器的就绪等待，仍由SCAN自身硬双圆检查；
- `m20_safe=true`：保留双话题`CLEAR`失效关闭，真机入口仍固定使用该模式。

这只修正启动图的一致性，不修改SCAN规划或控制算法。

## 配置切换

日常完整入口无需额外参数，默认加载 M20 几何档。精确复现上游几何时显式传入：

```bash
ros2 launch m20_warehouse_inspection inspection_mission_mujoco.launch.py \
  clearance_config:=/home/virdyn/robodog_nav_system/install/\
m20_scan_navigation/share/m20_scan_navigation/config/clearance_vendor.yaml
```

## 验证结果

- 相关静态回归：`47 passed`；最终五包逐包源测试合计`225 passed`；
- `m20_scan_navigation`和`m20_warehouse_inspection`最终重新构建成功；
- RViz原生SCAN：`(-37,0)`至`(-35,0)`，末端误差约`0.031 m`，无A-star/inside
  obstacle错误；
- 完整官方SDK + MuJoCo直达复验：从`(-18,0)`穿过`0.90 m`门洞到
  `(-7.021,-0.046)`，二维末端误差约`0.050 m`，障碍接触计数/事件/峰值力均为
  `0/0/0 N`，后端故障为空；
- 正式Action冷启动复验第一次因遗留的第二套MuJoCo/SDK进程争用而跌倒，确认并清理
  该遗留实例后，再启动被执行环境授权审查中断。因此该项记为“待正常终端补跑”，
  不把它写成通过，也不影响上一条已完成的动力学穿越证据。
