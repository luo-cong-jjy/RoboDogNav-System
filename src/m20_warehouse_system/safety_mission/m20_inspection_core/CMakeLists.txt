# ============================================================================
# 【文件职责】CMakeLists.txt —— m20_inspection_core 包的构建配置（ament_cmake）
# 本文件定义 m20_inspection_core（M20 仓库巡检安全与任务控制核心包）的构建规则：
#   1) 安装 Python 包源码（ament_python_install_package）；
#   2) 安装 config 配置目录到 share/<包名> 下；
#   3) 安装 4 个可执行脚本（碰撞守卫/楼层切换管理/任务执行器/安全监督）
#      到 lib/<包名> 下；
#   4) 安装 LICENSE 与 README.md；
#   5) 构建测试时注册 4 个 pytest 测试（策略纯函数单元测试）。
# ============================================================================

cmake_minimum_required(VERSION 3.8)  # 最低 CMake 版本要求
project(m20_inspection_core)         # 工程名（与 ROS2 包名一致）

find_package(ament_cmake REQUIRED)          # ament_cmake 基础构建支持
find_package(ament_cmake_python REQUIRED)   # ament Python 包安装支持

ament_python_install_package(${PROJECT_NAME})  # 安装 Python 包（m20_inspection_core 目录）

install(  # 安装 config 配置目录（yaml 参数文件）
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(  # 安装可执行脚本（启动入口）
  PROGRAMS
    scripts/m20_collision_guard      # 独立碰撞守卫节点入口
    scripts/m20_floor_switch_manager # 楼层切换管理节点入口
    scripts/m20_mission_executor     # 任务执行器节点入口
    scripts/m20_safety_supervisor    # 安全监督节点入口
  DESTINATION lib/${PROJECT_NAME}
)

install(  # 安装许可证与说明文档
  FILES LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)  # 构建测试模式：注册 pytest 单元测试
  find_package(ament_cmake_pytest REQUIRED)  # pytest 测试框架支持
  find_package(ament_lint_auto REQUIRED)     # 自动代码风格检查
  ament_lint_auto_find_test_dependencies()   # 自动发现 lint 测试依赖
  ament_add_pytest_test(   # 注册安全策略单元测试
    test_safety_policy
    test/test_safety_policy.py
  )
  ament_add_pytest_test(   # 注册碰撞预判策略单元测试
    test_collision_policy
    test/test_collision_policy.py
  )
  ament_add_pytest_test(   # 注册楼层切换策略单元测试
    test_floor_switch_policy
    test/test_floor_switch_policy.py
  )
  ament_add_pytest_test(   # 注册任务配置解析单元测试
    test_mission_policy
    test/test_mission_policy.py
  )
endif()

ament_package()  # 生成 ament 包元数据（包导出）
