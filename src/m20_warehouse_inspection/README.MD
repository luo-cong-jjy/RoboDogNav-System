主inspection_mission_mujoco.launch
    ↓参数透传
├─ inspection_mission_rviz.launch.py 【导航任务层】
│     → SCAN规划器，输出机身期望速度(vx,vy,wz)
│     → 把速度指令ROS话题发出
│                        phase4主launch（当前文件）
│                            ↓参数透传
│                        ├─ include multifloor_scan_rviz.launch.py
│                        │        ↓OpaqueFunction运行时读取yaml，动态生成节点
│                        │        ├─ Node m20_flat_map_server            # 多层地图
│                        │        ├─ Node robot_state_publisher          # URDF模型发布
│                        │        ├─ Node static_transform_publisher     # map-world tf
│                        │        ├─ Node m20_vendor_sensing_state       # 感知状态
│                        │        ├─ Node m20_navigation_adapter         # 导航速度→步态SDK适配
│                        │        ├─ Node m20_safety_supervisor          # 整机安全监控
│                        │        ├─ Node m20_floor_switch_manager       # 楼层切换管理
│                        │        ├─ Node m20_collision_guard(二选一)    # 轨迹碰撞保护
│                        │        ├─ include f1_scan.launch.py           # SCAN规划导航栈
│                        │        │
│                        │        ├─【条件】Node m20_rviz_kinematic_backend # rviz模式才启动平面仿真
│                        │        └─【条件】Node rviz2                     # use_rviz=true才启动可视化
│                        │
│                        ├─ Node m20_mission_executor【必启】
│                        │     → 读取巡检任务配置，输出导航目标点
│                        │     → 驱动导航规划器执行巡检
│                        │
│                        └─ Node m20_phase4_acceptance【条件，run_acceptance=true】
│                            → 自动化验收测试，校验整套系统
│
├─ mujoco_backend.launch.py 【物理仿真层】
│     → MuJoCo物理引擎，M20机器人动力学仿真
│     → 输出仿真世界的odom、机器人真实位姿
│
└─ sdk_locomotion.launch.py 【四足步态控制层】
      → 接收导航输出的机身速度
      → RL策略计算四条腿关节目标角度
      → 将关节指令发给MuJoCo后端执行