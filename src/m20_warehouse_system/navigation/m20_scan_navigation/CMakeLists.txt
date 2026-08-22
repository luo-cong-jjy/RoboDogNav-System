# =============================================================================
# CMakeLists.txt —— m20_scan_navigation 包构建脚本
# 所属模块：m20_scan_navigation（ament_cmake + ament_python 混合包）
# 职责：安装 Python 包（m20_scan_navigation）、脚本（scripts/）、配置与 launch
#       资源，并注册两个 pytest 回归测试。注意本包不编译任何 C++ 源码
#       （SCAN 核心算法在 m20_scan_planner 包中）。
# =============================================================================
cmake_minimum_required(VERSION 3.8)
project(m20_scan_navigation)

# 查找构建依赖：ament_cmake 基础 + Python 包安装支持
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装 Python 包（m20_scan_navigation/ 下的 .py 模块，含 __init__.py）
ament_python_install_package(${PROJECT_NAME})

# 安装可执行脚本（入口点：净空报告与导航网关）
install(
  PROGRAMS
    scripts/m20_clearance_report      # 净空分析报告脚本
    scripts/m20_navigation_gateway    # 导航网关入口脚本
  DESTINATION lib/${PROJECT_NAME}
)
# 安装配置与 launch 资源（运行时按 share 目录加载）
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
# 安装文档/许可文件
install(
  FILES LICENSE NOTICE README.md
  DESTINATION share/${PROJECT_NAME}
)

# 构建测试（colcon test 时启用）
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 导航契约回归测试（话题/安全边界/厂商等价性）
  ament_add_pytest_test(
    test_navigation_contract
    test/test_navigation_contract.py
  )
  # 净空档位回归测试
  ament_add_pytest_test(
    test_clearance
    test/test_clearance.py
  )
endif()

# 生成 ament 包配置（声明依赖、导出等）
ament_package()
