# M20 实机导航部署简明流程

## 启动前

- 外部主机已连接 M20 网络，Basic Server 地址和端口已确认；
- 车载自主控制源已停止，外部主机拥有控制权；
- 定位、点云、实测速度和实体急停均可用。

## 运动预检

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch m20_warehouse_inspection inspection_mission_hardware.launch.py \
  command_ownership_confirmed:=true \
  auto_enable_motion:=false
ros2 launch m20_hardware_preflight m20_hardware_preflight.launch.py
ros2 service call /m20_hardware_preflight/start_motion_test std_srvs/srv/Trigger '{}'
```

预检失败时不得启动导航。首次仅测试低速直线和停止。

## 导航前确认

确认以下状态持续更新：

```text
/m20/locomotion/backend_ready
/m20/locomotion/backend_fault
/m20/locomotion/measured_twist
/m20/control/safety_state
```

同时确认碰撞保护为 `CLEAR`，定位和点云没有 stale/timeout。

## 接入导航

通过 RViz 发布近距离、无遮挡目标点，按直线、转弯、绕障、重规划顺序逐步验收。

```text
SCAN 规划器 -> 安全监督 -> m20_locomotion_manager -> Basic Server -> M20
```

## 安全约束

- `auto_enable_motion` 保持 `false`，由人工确认后使能；
- 后端、指令、状态或定位超时必须停车；
- 碰撞保护不可关闭；
- 急停后必须人工确认并重新使能；
- 不得同时运行车载和外部两个自主控制源。
