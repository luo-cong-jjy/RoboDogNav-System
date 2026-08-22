# ======================================================================
# CMakeLists.txt —— m20_warehouse_sim 包构建脚本（中文注释版）
# 功能：安装 Python 包（ament_cmake_python）、配置文件、可执行脚本与文档，
#       并注册 pytest 单元测试
# 构建方式：colcon build --packages-select m20_warehouse_sim
# 说明：本文件只新增中文注释，未改动任何原始内容
# ======================================================================

# 最低 CMake 版本要求（3.8 及以上）
cmake_minimum_required(VERSION 3.8)
# 工程名称：m20_warehouse_sim（纯 Python 包）
project(m20_warehouse_sim)

# 查找 ament_cmake（ROS2 CMake 构建基础）
find_package(ament_cmake REQUIRED)
# 查找 ament_cmake_python（ROS2 Python 包安装支持）
find_package(ament_cmake_python REQUIRED)

# 安装 Python 包本体（m20_warehouse_sim/m20_warehouse_sim/ 目录）
ament_python_install_package(${PROJECT_NAME})

# 安装 config 配置目录（RViz 后端参数 YAML）到 share/m20_warehouse_sim
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行脚本（带可执行权限）到 lib/m20_warehouse_sim
install(
  PROGRAMS scripts/m20_rviz_kinematic_backend   # RViz 平面运动学后端节点入口脚本
  DESTINATION lib/${PROJECT_NAME}
)

# 安装文档与许可证到 share/m20_warehouse_sim
install(
  FILES LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)

# 构建测试时（BUILD_TESTING 由 colcon 默认开启）：
if(BUILD_TESTING)
  # 查找 pytest 测试支持
  find_package(ament_cmake_pytest REQUIRED)
  # 查找自动 lint 测试框架
  find_package(ament_lint_auto REQUIRED)
  # 自动发现并注册 lint 测试依赖
  ament_lint_auto_find_test_dependencies()
  # 注册平面运动学函数的 pytest 测试
  ament_add_pytest_test(
    test_kinematics
    test/test_kinematics.py
  )
endif()

# 生成 ROS2 包所需的配置与注册信息（colcon build 收尾步骤，必须调用）
ament_package()
