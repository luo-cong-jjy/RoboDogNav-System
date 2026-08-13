# M20 RViz 运动学后端

本包提供不依赖 Gazebo 的二维刚体运动学后端，用于先完成 RViz、SCAN 和安全控制闭环。

运动后端唯一速度输入：

```text
/m20/control/cmd_vel_safe
```

输出：

```text
/m20/sim/body_pose
/m20/sim/path
/m20/sim/backend_ready
/joint_states
map -> base_link TF
```

`/m20/sim/body_pose` 遵循 ROS `Odometry` 坐标契约：pose 位于 `map`，
twist 位于 `base_link`。内部保留的 `vx_world/vy_world` 只用于平面位姿积分，
不得作为控制器的机体系速度反馈。

阶段 3 提供 `/m20/sim/set_pose` 服务。它校验有限数值后原子更新平面位姿，清零速度和
残留命令，重置轨迹，并立即发布确认后的 odom/TF。该接口只属于 RViz 仿真 profile；
实机后端必须拒绝 teleport，并用定位后端重初始化取代它。

该节点不是动力学仿真，也不模拟轮地接触。Gazebo 后端将在后续阶段实现。
