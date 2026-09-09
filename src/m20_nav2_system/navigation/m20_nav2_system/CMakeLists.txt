cmake_minimum_required(VERSION 3.8)  # 声明最低 CMake 版本要求（3.8）
project(m20_nav2_system)  # 工程名：m20_nav2_system（必须与包名/package.xml 一致）


# Gazebo Classic's package config emits an internal FindPkgConfig developer
# warning with newer CMake versions. It is unrelated to this package.
# （抑制第三方 CMake 开发警告：新版 CMake 下 Gazebo Classic 的包配置会
#   产生内部 FindPkgConfig 开发警告，与本包代码无关）
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS TRUE CACHE BOOL "Suppress third-party CMake developer warnings")  # 全局关闭 CMake 开发警告开关

# —— 查找依赖包（全部 REQUIRED：缺失任一包即编译失败）——
find_package(ament_cmake REQUIRED)  # ament 构建系统核心（提供 ament_target_dependencies、install、ament_package 等）
option(BUILD_GAZEBO_PLUGINS "Build Gazebo-only simulation plugins" ON)
if(BUILD_GAZEBO_PLUGINS)
  find_package(gazebo REQUIRED)  # Gazebo-only dependency
endif()
find_package(builtin_interfaces REQUIRED)  # 内置消息接口（如 Time、Duration 等基础类型）
find_package(geometry_msgs REQUIRED)  # 几何消息（Point、Pose、Twist、TransformStamped 等）
find_package(nav_msgs REQUIRED)  # 导航消息（Odometry、OccupancyGrid、Path 等）
find_package(rclcpp REQUIRED)  # ROS2 C++ 客户端库（编写 ROS2 节点/插件的核心库）
find_package(sensor_msgs REQUIRED)  # 传感器消息（LaserScan、PointCloud2、JointState 等）
find_package(tf2_ros REQUIRED)  # TF2 坐标变换库（发布/监听坐标变换）

# —— 设置 C++ 编译标准 ——
set(CMAKE_CXX_STANDARD 17)  # 使用 C++17 标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 强制要求编译器支持 C++17（不支持则报错，不自动降级）

# —— 编译 4 个 Gazebo 插件（动态共享库）——
if(BUILD_GAZEBO_PLUGINS)
foreach(plugin sinusoidal_obstacle waypoint_obstacle four_wheel_drive legged_kinematic)  # Gazebo-only plugins
  add_library(m20_${plugin}_plugin SHARED src/${plugin}_plugin.cpp)  # 每个插件编译为共享库（src/ 下同名 .cpp）
  target_include_directories(m20_${plugin}_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})  # 添加 Gazebo 头文件搜索路径（私有）
  target_link_libraries(m20_${plugin}_plugin ${GAZEBO_LIBRARIES})  # 链接 Gazebo 库
endforeach()

ament_target_dependencies(m20_four_wheel_drive_plugin builtin_interfaces rclcpp geometry_msgs nav_msgs tf2_ros)  # 四轮驱动插件：声明其依赖的 ROS2 消息/库
ament_target_dependencies(m20_legged_kinematic_plugin builtin_interfaces rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros)  # 足式运动插件：声明其依赖的 ROS2 消息/库（比前者多 sensor_msgs）
endif()

# —— 编译演示可执行程序 ——
add_executable(line_demo src/line_demo.cpp)  # 直线运动演示程序
ament_target_dependencies(line_demo rclcpp geometry_msgs)  # 声明 line_demo 的 ROS2 依赖

add_executable(circle_demo src/circle_demo.cpp)  # 圆周运动演示程序
ament_target_dependencies(circle_demo rclcpp geometry_msgs)  # 声明 circle_demo 的 ROS2 依赖

# —— 安装规则 ——
if(BUILD_GAZEBO_PLUGINS)
install(TARGETS  # 安装 Gazebo 插件共享库
  m20_sinusoidal_obstacle_plugin m20_waypoint_obstacle_plugin
  m20_four_wheel_drive_plugin m20_legged_kinematic_plugin
  LIBRARY DESTINATION lib)  # 动态库安装到安装前缀的 lib 目录（供 Gazebo 插件路径加载）
endif()

install(TARGETS line_demo circle_demo  # 安装两个演示可执行程序
  DESTINATION lib/${PROJECT_NAME})  # 安装到 lib/m20_nav2_system 目录

install(DIRECTORY launch config docs  # 安装 launch 脚本、配置参数、文档目录
  DESTINATION share/${PROJECT_NAME})  # 安装到 share/m20_nav2_system（运行时经 get_package_share_directory 查找）

install(DIRECTORY maps models rviz worlds  # 安装资源目录：地图、模型、RViz 配置、Gazebo 世界文件
  DESTINATION share/${PROJECT_NAME})  # 安装到 share/m20_nav2_system

install(FILES README.md ${CMAKE_CURRENT_SOURCE_DIR}/../../hardware_dependencies.txt  # 安装说明与实机依赖清单
  DESTINATION share/${PROJECT_NAME})  # 安装到 share/m20_nav2_system

install(PROGRAMS  # 安装 Python 脚本（安装时自动赋予可执行权限）
  scripts/factory_inspection_nav2_mission scripts/factory_inspection_nav2_mission_gazebo
  scripts/factory_inspection_nav2_mission_mujoco
  scripts/goal_pose_restamper scripts/goal_pose_restamper_gazebo  # 目标点时间戳重写脚本
  scripts/m20_standing_joint_state_publisher scripts/m20_standing_joint_state_publisher_gazebo  # 站立形态关节状态发布脚本
  scripts/pcd/grid_astar_planner scripts/pcd/lifecycle_configure_activate  # PCD 栅格 A* 规划器、生命周期配置/激活脚本
  scripts/pcd/pcd_to_occupancy_grid scripts/pcd/pcd_slice_publisher  # PCD 转占用栅格脚本、PCD 切片发布脚本
  scripts/m20_grid_lidar_simulator  # 后端无关的二维栅格雷达模拟器
  scripts/m20_gazebo_dynamic_obstacle_tracker  # Gazebo 实测扫描动态障碍跟踪与预测
  scripts/m20_mujoco_odom_adapter  # MuJoCo body pose 到 Nav2 odom/TF 的适配
  scripts/m20_factory_scene_markers  # 同源 SDF 障碍物的 RViz 三维显示
  scripts/m20_nav2_goal_bridge  # RViz PoseStamped 到 Nav2 action 的显式桥接
  scripts/m20_dynamic_safety_filter  # 动态障碍 TTC 速度安全过滤器
  scripts/m20_keyboard_teleop scripts/m20_keyboard_teleop_gazebo  # Gazebo SLAM 建图时的基础键盘速度控制
  scripts/gazebo_world_to_mujoco_geoms.py  # SDF 静态碰撞到 MuJoCo 几何转换器
  scripts/build_m20_mujoco_factory_world.py  # 注入官方 M20 MJCF 模板
  scripts/validate_m20_mujoco_factory_world.py  # 生成 MJCF 资产校验器
  DESTINATION lib/${PROJECT_NAME})  # 全部安装到 lib/m20_nav2_system

ament_package()  # 生成 ament 包元数据：导出 share 目录、生成包索引（必须在所有 install 之后调用）
