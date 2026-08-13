# PCD-only 地图与原生 SCAN 单一路线迁移记录

日期：2026-08-11  
范围：仿真基线收敛，为后续 Foxy/x86 实机迁移减少非必要接口  
结论：完成

## 1. 修改原因

项目早期为了验证平面双区域和长距离绕障，额外生成了 PGM/YAML，并在 SCAN 外增加了
静态二维 A*、路线简化和顺序短子目标。这套链路与原项目
`src/third_party/SCAN-Planner` 的运行效果不一致，也给真实 LIO PCD 引入了不必要的
离线地图转换步骤。

本轮确定以下现行原则：

1. 每层持久化地图只保留 PCD 和 JSON 元数据；
2. 人工目标和自动任务只使用原生 SCAN GridMap、rebound、B-spline、controller 和
   可视化；
3. 独立 M20 安全层继续存在，但只消费 SCAN 在线占据点云，不建立第二套规划器；
4. 地图切换仍由 `floor_id + generation` 事务控制，只切换 active PCD，不改变机器人
   位姿；
5. 实机只需把仿真 PCD 来源换为实测静态 PCD，把实时感知/位姿来源换为
   Elevator-LIO，导航与安全接口保持不变。

## 2. 现行数据链

```text
磁盘 PCD + JSON
  -> m20_flat_map_server
  -> /map_generator/global_cloud（当前活动楼层）
  -> pcl_render_node（仿真）/ 实机 LIO 点云
  -> /quad_0/cloud
  -> SCAN 3D GridMap
      -> rebound A* / B-spline / 原版可视化
      -> /grid_map/occupancy (sensor_msgs/PointCloud2)
           -> collision_guard 内存局部栅格

/move_base_simple/goal
  -> native SCAN
  -> /m20/navigation/cmd_vel_raw
  -> M20 rolling adapter
  -> collision guard + safety supervisor
  -> /m20/control/cmd_vel_safe
  -> RViz kinematic / MuJoCo / factory backend
```

`/grid_map/occupancy` 名称虽然包含 occupancy，但消息类型是实时
`sensor_msgs/PointCloud2`，不是 `nav_msgs/OccupancyGrid`，也不对应任何 PGM 文件。

## 3. 代码修改

### 3.1 地图资产与服务器

- `map_assets.py` 只写 ASCII PCD 和 JSON；内部栅格只用于生成时的连通性断言，不写盘；
- `map_server_node.py` 删除 PGM/YAML 读取和 active OccupancyGrid 发布；
- 15 份系统/测试配置删除 `occupancy_file`；
- 删除 maps 下 56 个 PGM/YAML 源资产，重新生成 28 份 PCD 对应元数据；
- 冻结基线更新为 PCD-only SHA-256；
- RViz 删除 active OccupancyGrid 和项目新增全局 A* 路线显示。

### 3.2 导航

- 删除 `m20_grid_route_planner` executable、`grid_route.py`、`grid_route_node.py`、
  `f1_grid_route.yaml` 及单元测试；
- `f1_scan.launch.py` 不再创建二维路线节点，目标直接进入
  `/move_base_simple/goal`；
- gateway 删除 `use_grid_route`、`collision_grid_route_enabled`、route state、route
  reset、routed publisher 和顺序子目标分支；
- 人工目标保持原项目直连语义；typed Action 仍校验楼层/代次并负责取消、超时和 reset；
- 有界碰撞恢复耗尽后只允许“reset SCAN -> 等待 guard CLEAR -> 同目标原生重规划”，
  超过次数返回 NO_ROUTE；
- 删除已无调用者的 rear-route policy 和 boundary-recovery 专用探针。

### 3.3 在线点云安全

- collision guard 订阅 `/grid_map/occupancy` PointCloud2；
- 按当前机身高度构建 16 m × 16 m、0.10 m 分辨率的机器人中心内存栅格；
- 只投影 `body_z-0.50 m` 到 `body_z+0.70 m` 的占据体素，过滤地面和高处点；
- 保留统一能力配置中的 M20 前后双圆足迹、滚动扫掠与有界恢复；
- 点云缺失、帧错误或超过 0.75 s 未更新时 fail closed；
- 删除 safety supervisor 中已无生产者的 `route_segment_hold` 输入。

### 3.4 文档与实机路径

- 更新接口契约、总 README、导航/安全 README、仿真模块清单和实机逐步部署文档；
- 实机地图准备改为 `ASCII PCD + JSON`，不再要求生成 PGM/YAML；
- 历史开发日志保留原试验记录，`system_plan.md` 顶部增加现行方案覆盖声明。

## 4. 兼容性与限制

- `NavigateFloor.Feedback.route_update_count` 字段为冻结接口，暂不破坏；原生模式下该值
  记录 SCAN 最终目标发布次数；
- SCAN 规划参数仍来自 vendor profile，本轮没有改变 `0.25 m` 双圆硬碰撞半径、
  `0.20 m` 优化距离或原版可视化；
- collision guard 的内存栅格是执行安全近场视图，不负责产生全局绕障路线；
- 真机导入器当前仍只接受 ASCII `FIELDS x y z intensity` PCD。Elevator-LIO 若输出
  binary PCD，现场导入前仍需离线清理和格式转换；
- 本轮动态验收覆盖 RViz 平面后端的短距离单目标，不替代后续 MuJoCo 多目标和真实 F1
  五目标验收。

## 5. 增量安装空间说明

源码和 CMake 安装清单中已经没有旧 route/PGM 产物。历史 `--symlink-install` 目录可能
保留指向已删除源码的断链，但 `ros2 pkg executables m20_scan_navigation` 只列出
`m20_clearance_report` 与 `m20_navigation_gateway`。新建实机工作空间重新编译时不会生成
这些断链；部署包必须来自干净构建，不复制开发机旧 `install/`。

