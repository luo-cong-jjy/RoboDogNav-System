# m20_warehouse_interfaces

本包只定义 M20 仓库巡检系统的跨模块 ROS 2 接口，不包含业务实现。独立接口包可以避免地图、仿真、导航和任务管理模块之间形成循环依赖。

核心约束：

- `generation` 是活动楼层地图的单调递增代际号。
- 地图切换开始时 `FloorState.ready=false`，提交成功后才变为 `true`。
- `SwitchFloor` 失败或取消时，上层控制必须继续保持停车。
- `LocalSensingState.ready=true` 只表示当前楼层、当前代际已经产生新鲜局部点云。
- `LocalSensingState` 同时携带单帧点数与 XY 范围，用于轻量检查跨层残留。

阶段 4 增加：

- `NavigationState` 与 `NavigateFloor`：把目标绑定到当前 floor/generation。
- `MissionState` 与 `RunMission`：反馈任务 step、暂停和故障状态。
- `ControlMission`：控制活动任务的 pause、resume、stop 和 retry-current。

所有长事务使用 Action；点云仍使用标准 `PointCloud2`，但有效性由 typed
`floor_id + generation` 状态约束。
