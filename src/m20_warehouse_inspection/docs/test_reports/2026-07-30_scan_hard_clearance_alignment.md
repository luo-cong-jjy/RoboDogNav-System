# SCAN 硬规划净空对齐测试报告

日期：2026-07-30

> 状态：对应 `0.30/0.15 m` SCAN 参数已撤回，以下结果仅作历史对照。

## 配置

```yaml
grid_map.double_cylinder_radius: 0.30
optimization.dist0: 0.15
footprint_radius: 0.25
safety_margin: 0.05
```

规划硬包络与执行保护包络均为 `0.30 m`，硬半径加软距离仍为
`0.45 m`。独立 vendor 配置保持 `0.25/0.20 m`。

## 自动验证

```text
src/m20_scan_navigation/test
src/m20_warehouse_inspection/test

91 passed
```

两个修改包使用 `--symlink-install` 重新构建成功。安装空间读取结果：

```text
hard radius:      0.30 m
soft clearance:   0.15 m
nominal envelope: 0.45 m
guard envelope:   0.30 m
```

`m20_clearance_report --profile conservative --map-resolution 0.05` 结果：

```text
native_hard=0.60m
native_nominal=0.90m
0.75m MARGINAL
0.80m MARGINAL
0.90m NOMINAL_CANDIDATE
```

## 动态证据

工作区已有同一硬/软拆分的官方策略 MuJoCo 门洞证据：

```text
artifacts/navigation_motion/
  2026-07-30_heading_alignment_narrow_090_tracking_v1/
    clearance_summary.json
```

结果为：

- `31.20699 s` 完成；
- 最终误差 `0.15358 m`；
- 最小保护净空 `0.07585 m`；
- 预测/当前保护事件 `0/0`；
- MuJoCo 实体障碍接触 `0`。

## 当前限制

本轮速度对齐后的新冷启动复验在自动执行环境中被 DDS socket 权限阻止：

```text
TRANSPORT_UDP Error: Error creating socket: Operation not permitted
```

因此本报告不把离线连通性或旧动态证据冒充为本轮新组合的动态通过。正常本机环境
仍需执行一次 `0.90 m` 门洞和完整高密度巡检；验收标准是任务完成、无
`CURRENT_FOOTPRINT`、无 MuJoCo 障碍接触。
