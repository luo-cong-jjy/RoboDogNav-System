# ============================================================
# CMakeLists.txt —— m20_scan_planner 包的构建脚本（ament_cmake）
# 职责：
#   1. 声明构建类型（Release）与 C++17 标准，开启警告；
#   2. 查找全部依赖包（bspline_opt、plan_env、scan_planner_msgs 等）；
#   3. 编译规划器主节点 scan_planner_node（scan_planner_node.cpp /
#      scan_replan_fsm.cpp / planner_manager.cpp）以及
#      open_loop_controller / closed_loop_controller / m20_kinematic_sim /
#      m20_gait_publisher 四个辅助可执行文件；
#   4. 安装可执行文件、头文件、launch/config/models 目录与 Python 脚本；
#   5. 注册两个 launch 测试（启动冒烟测试与路点参数测试）。
# ============================================================
cmake_minimum_required(VERSION 3.8) # 要求 CMake 最低版本 3.8
project(m20_scan_planner) # 工程名：m20_scan_planner

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # Match the vendored SCAN deployment contract when a workspace build does
  # not pass an explicit build type.
  # 当工作区构建未显式指定构建类型时，与内置 SCAN 部署约定保持一致：
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE) # 强制设为 Release
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # 未指定时默认使用 C++17 标准
endif()
add_compile_options(-Wall -Wextra -Wpedantic) # 开启常用编译警告

# ---- 查找构建依赖包 ----
find_package(ament_cmake REQUIRED) # ament_cmake：ROS 2 构建系统
find_package(ament_index_cpp REQUIRED) # ament 资源索引（C++ 接口）
find_package(bspline_opt REQUIRED) # B 样条轨迹优化库（SCAN-Planner 核心）
find_package(Eigen3 REQUIRED) # Eigen 线性代数库
find_package(geometry_msgs REQUIRED) # 几何消息类型
find_package(m20_warehouse_interfaces REQUIRED) # M20 自定义消息接口
find_package(nav_msgs REQUIRED) # 导航消息（Odometry/Path 等）
find_package(path_searching REQUIRED) # 路径搜索库（A* 等）
find_package(plan_env REQUIRED) # 规划环境（地图/网格表示）
find_package(rclcpp REQUIRED) # ROS 2 C++ 客户端库
find_package(scan_planner_msgs REQUIRED) # SCAN-Planner 自定义消息
find_package(sensor_msgs REQUIRED) # 传感器消息（点云等）
find_package(std_msgs REQUIRED) # 标准消息
find_package(tf2 REQUIRED) # TF2 基础库
find_package(tf2_geometry_msgs REQUIRED) # TF2 与几何消息转换
find_package(tf2_ros REQUIRED) # TF2 ROS 接口
find_package(traj_utils REQUIRED) # 轨迹工具库
find_package(visualization_msgs REQUIRED) # 可视化消息（Marker 等）

# 规划器主节点依赖的包列表（供 ament_target_dependencies 使用）
set(scan_planner_dependencies
  ament_index_cpp bspline_opt geometry_msgs m20_warehouse_interfaces nav_msgs path_searching plan_env
  rclcpp scan_planner_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
  traj_utils visualization_msgs)

# ---- 编译规划器主节点 ----
add_executable(scan_planner_node
  src/scan_planner_node.cpp # 节点入口：参数声明与主循环
  src/scan_replan_fsm.cpp   # 有限状态机：控制规划/重规划流程
  src/planner_manager.cpp)  # 规划管理器：调用全局/局部规划器
target_include_directories(scan_planner_node PRIVATE include) # 添加头文件搜索路径
ament_target_dependencies(scan_planner_node ${scan_planner_dependencies}) # 声明依赖
target_link_libraries(scan_planner_node Eigen3::Eigen) # 链接 Eigen

# ---- 编译四个辅助可执行文件（开环/闭环控制器、运动学仿真、步态发布） ----
foreach(target open_loop_controller closed_loop_controller m20_kinematic_sim m20_gait_publisher)
  add_executable(${target} src/${target}.cpp) # 每个目标对应 src/ 下的同名源文件
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    bspline_opt geometry_msgs nav_msgs rclcpp scan_planner_msgs sensor_msgs
    std_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(${target} Eigen3::Eigen)
endforeach()

# ---- 安装规则 ----
install(TARGETS
  scan_planner_node open_loop_controller closed_loop_controller
  m20_kinematic_sim m20_gait_publisher
  DESTINATION lib/${PROJECT_NAME}) # 可执行文件安装到 lib/<包名>
install(DIRECTORY include/ DESTINATION include) # 安装头文件
install(DIRECTORY launch config models DESTINATION share/${PROJECT_NAME} OPTIONAL) # 安装 launch/config/models（可选）
install(FILES launch/default.rviz DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL) # 安装 RViz 配置（可选）
install(PROGRAMS
  scripts/keypoint_recorder.py                 # 路点录制脚本
  scripts/collada_mesh_to_pcd.py               # Collada 网格转 PCD 脚本
  scripts/generate_single_floor_gazebo_map.py  # 生成单层 Gazebo 地图脚本
  scripts/traversability_global_planner.py     # 可通行性全局规划器脚本
  scripts/support_path_follower.py             # 支撑路径跟踪脚本
  DESTINATION lib/${PROJECT_NAME} OPTIONAL)    # Python 脚本作为可执行程序安装

# ---- 测试 ----
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED) # launch 测试支持
  add_launch_test(test/test_planner_startup.py TIMEOUT 30) # 启动冒烟测试（30 秒超时）
  add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30) # 路点参数测试（30 秒超时）
endif()

ament_package() # ament 打包（生成包描述与环境钩子）
