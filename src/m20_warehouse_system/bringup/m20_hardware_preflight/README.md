# M20 实机运动预检

本包用于在接入 SCAN 导航前，单独检查 M20 实机运动接口和安全链路。

默认只发布零速度，不自动使能运动。确认后端就绪、无故障、安全状态正常且实测速度反馈新鲜后，才允许显式触发低速测试。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch m20_hardware_preflight m20_hardware_preflight.launch.py
ros2 service call /m20_hardware_preflight/start_motion_test std_srvs/srv/Trigger '{}'
```

测试以 `0.05 m/s` 前进约 1 秒，然后自动归零。首次测试建议抬高机器狗或在空旷区域进行，并安排人员随时使用实体急停。

通过条件：后端已就绪、故障为空、安全状态为 `NAVIGATION` 或 `MANUAL`、实测速度持续更新。预检通过不等于完成导航验收，仍需验证点云、定位、碰撞保护和急停链路。
