# 2026-07-30 SCAN 硬规划层与执行保护层对齐

> 状态：已撤回。当前主流程恢复原项目 `0.25 m` 硬半径和 `0.20 m` 优化器
> 距离；额外 `0.05 m` 只在下游执行保护器中使用。本文仅保留为试验过程记录。

## 复现

完整 MuJoCo 系统在高密度场景中再次出现：

```text
The robot is inside an obstacle
First three control points are in obstacles
A-star error
```

对应起点约为 `(-28.4, -1.25, 0.565)`。上一轮把 SCAN 轨迹生成、优化和闭环
执行速度统一为 `0.40 m/s` 后，时间参数化不再快于底层执行，但仍不能阻止一条
几何上紧贴硬占据边界的轨迹被发布。

## 原因

正式配置此前为：

```yaml
grid_map.double_cylinder_radius: 0.25
optimization.dist0: 0.20
```

独立命令保护器的有效半径却是：

```text
0.25 m footprint + 0.05 m safety margin = 0.30 m
```

因此 `0.25～0.30 m` 区域对 SCAN 硬碰撞判定是自由空间，对执行保护器却是危险
区域。`optimization.dist0` 也不是全局距离场：原生 rebound 实现只给已识别碰撞段
建立推出方向，不能保证所有“无碰撞但贴边”的轨迹都兑现完整软距离。

## 修改

完整系统的 `clearance_conservative.yaml` 改为：

```yaml
grid_map.double_cylinder_radius: 0.30
optimization.dist0: 0.15
```

这样：

- SCAN 硬规划层与执行保护器均为 `0.30 m`；
- 单侧名义包络仍为 `0.30 + 0.15 = 0.45 m`；
- `0.90 m` 正式最小通道仍保留中心可行解；
- vendor 独立 profile 继续保持原项目的 `0.25/0.20 m`，便于算法对照。

没有修改 M20 的物理尺寸、保护器安全余量、地图分辨率、重规划算法或官方
MuJoCo/ONNX 运动模型。

## 验收

配置契约要求：

1. conservative 硬半径必须等于保护器机身半径与安全余量之和；
2. 硬半径与软距离之和必须保持 `0.45 m`；
3. `0.90 m` 门洞必须在原生 SCAN、官方策略和 MuJoCo 动力学链中完成，无当前
   足迹停车和实体障碍接触。

## 本轮验证状态

- 导航与集成功能包测试：`91 passed`；
- `m20_scan_navigation`、`m20_warehouse_inspection` 重新构建成功；
- 安装空间确认硬/软/保护包络分别为 `0.30/0.15/0.30 m`；
- 离线净空报告将 `0.90 m` 判定为 `NOMINAL_CANDIDATE`；
- 工作区已有同一 `0.30/0.15 m` 包络的 MuJoCo 门洞证据：`31.21 s` 完成，
  最小连续保护净空 `0.0759 m`，无保护事件和实体接触。

加入本轮 `0.40 m/s` 速度对齐后的新动态冷启动曾尝试执行，但当前自动执行沙箱
禁止 DDS 创建本地 UDP socket，ROS 2 参与者无法建立。该环境错误不计为通过或
失败，仍需在正常本机 DDS 环境复验。
