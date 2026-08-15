# 2026-08-13 实机部署前可执行性核对

## 核对范围

- `04_m20_pao_step_by_step_deployment.md` 总部署手册；
- `src/Elevator-LIO/Virdy-m20-pro-建图定位启动.md` 包内建图/定位手册；
- Humble 开发机到 Foxy/Python 3.8 目标机的 launch、C++ 头文件、CLI 和构建边界；
- `deep-robotics-msg`、Elevator-LIO、外部 RoboSense 驱动、SCAN 和 M20 运动后端的放行顺序。

## 核对中发现并修正的问题

1. 文档仍指向已更名的 Elevator-LIO 手册路径；已统一为
   `src/Elevator-LIO/Virdy-m20-pro-建图定位启动.md`，并纳入 Foxy 预检。
2. 原发布流程整体替换目标机 `src`，却没有保留已验证的顶层
   `rslidar_sdk`/`rslidar_msg`；已增加从目标机原样纳入发布、单一驱动所有者、
   `ENABLE_TRANSFORM=ON` 构建和 `ros2 launch rslidar_sdk start.py` 启动检查。
3. `RELEASE_TAG` 在新终端丢失，且文档混用旧发布号；已统一示例，并由专用
   Foxy 环境文件导出。
4. 实场 PCD/site YAML 修改后只改了 source，hardware launch 仍会读旧 install；已补全
   `m20_warehouse_inspection` 重新安装和 install 资产存在性检查。
5. Humble 命名形式的 `static_transform_publisher` 参数不被 Foxy 支持；已改为两版
   共用的位置参数。
6. SCAN 适配层直接包含 Humble 后缀的 `tf2_geometry_msgs.hpp`/`tf2/utils.hpp`；
   已增加 Foxy `.h` 回退分支，未改变任何规划、跟踪或安全参数。
7. 导航闭包有 Python 3.9 的 `str.removeprefix()` 调用；已改为 Python 3.8 可执行的
   等价实现，并扩展预检防止回归。
8. Foxy `ros2 topic echo` 没有本手册原使用的 `--once`/`--field`；已改成
   `timeout + --no-arr` 命令，并为 transient-local 状态话题显式指定 QoS。
9. 软急停一次性发布者与后端的 transient-local 订阅者存在 QoS 错配风险；
   已补全触发、AOS 人工恢复、项目本地 `false` 清锁和 disable 顺序。

## 开发机验证

- `validate_foxy_hardware_source.py --transport basic_server`：PASS；
- `validate_foxy_hardware_source.py --transport direct_ros`：PASS；
- `deep-robotics-msg` ABI：与当前记录的 M20-PRO 端点一致；
- `m20_scan_planner` + `m20_warehouse_inspection` Humble 重新构建：PASS；
- `m20_scan_planner` 启动测试：2 tests，0 failure；
- `m20_warehouse_inspection`：216 tests，0 failure，1 skipped；
- 本轮专项 Python/配置测试：25 tests，0 failure；
- hardware launch `--show-args`：PASS；
- `inspection_mission_mujoco.launch.py use_rviz:=false use_mujoco_viewer:=false`
  35 s 一键无界面冒烟：14 个进程全部启动，两层地图预加载，官方 SDK 起立稳定，
  MuJoCo backend ready，首帧新鲜点云到达；
- 手册 Markdown 代码围栏：成对；`git diff --check`：PASS。

一键冒烟由 `timeout` 在 35 s 发送 SIGINT，因此返回 124；这是测试窗口到时，不是启动
失败。当前受限执行环境禁止创建 UDP 网络 socket，Fast DDS 会打印
`Operation not permitted`，但本机共享内存链路仍完成了上述启动门禁。强制收尾时
SCAN `pcl_render_node` 未在 5 s 内主动退出，被 launch 升级为 SIGTERM；本轮只据此
确认一键启动和数据面未回归，不把定时强制退出当作生产优雅关机验收。

## 仍需在目标机完成的门禁

本机没有 Ubuntu 20.04/Foxy 和 M20 实体硬件，因此不声称实机已验收。必须按总手册
G2→G8 顺序在背部主机完成 Foxy 原生构建、当前固件话题/QoS 复核、双雷达和 IMU、
Elevator-LIO 建图/重定位、只读联调、支撑架急停、空场低速和 F1 五目标导航。
任一级失败都不能跳级。
