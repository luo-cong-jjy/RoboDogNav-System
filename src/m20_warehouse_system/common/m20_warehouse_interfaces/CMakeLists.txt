cmake_minimum_required(VERSION 3.8)  # 最低 CMake 版本要求
project(m20_warehouse_interfaces)    # 项目名：仓库多楼层系统的 ROS2 接口定义包

# --- 查找依赖包 ---
find_package(ament_cmake REQUIRED)              # ament 构建系统
find_package(geometry_msgs REQUIRED)            # 几何消息（PoseStamped 等）
find_package(rosidl_default_generators REQUIRED) # ROSIDL 接口代码生成器
find_package(std_msgs REQUIRED)                 # 标准消息（Header 等）

# --- 生成全部自定义接口（msg/srv/action） ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FloorState.msg"           # 楼层地图状态
  "msg/LocalSensingState.msg"    # 局部感知就绪状态
  "msg/MissionState.msg"         # 巡检任务状态
  "msg/NavigationState.msg"      # 导航目标状态
  "srv/ResetNavigation.srv"      # 复位旧楼层导航状态服务
  "srv/ControlMission.srv"       # 任务控制（暂停/继续/停止/重试）服务
  "srv/SetSimulationPose.srv"    # 仿真瞬移位姿服务
  "srv/SwitchMap.srv"            # 楼层地图切换（乐观锁）服务
  "action/NavigateFloor.action"  # 楼层内导航动作
  "action/ExecuteFloorTransfer.action"  # 楼层转运提供者动作
  "action/RunMission.action"     # 运行巡检任务动作
  "action/SwitchFloor.action"    # 跨楼层切换动作
  DEPENDENCIES geometry_msgs std_msgs  # 接口中引用的外部消息包
)

# --- 导出运行依赖并打包 ---
ament_export_dependencies(rosidl_default_runtime)  # 导出运行期 ROSIDL 依赖
ament_package()                                    # 标准 ament 打包
