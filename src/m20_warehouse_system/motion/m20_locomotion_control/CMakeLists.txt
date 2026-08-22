# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：CMakeLists.txt
# 所属：m20_locomotion_control —— M20 运动控制包构建脚本（ament_cmake）
# 核心职责：
#   - 安装 Python 包（ament_python_install_package）；
#   - 安装 config/launch 目录到 share；
#   - 安装 5 个可执行脚本到 lib；
#   - 安装 LICENSE/README；
#   - 声明 7 个 pytest 测试（BUILD_TESTING 时）。
# ============================================================================
cmake_minimum_required(VERSION 3.8)  # 最低 CMake 版本
project(m20_locomotion_control)      # 项目名

find_package(ament_cmake REQUIRED)         # 查找 ament_cmake（构建基础）
find_package(ament_cmake_python REQUIRED)  # 查找 Python 包安装支持

ament_python_install_package(${PROJECT_NAME})  # 安装 Python 包（m20_locomotion_control）

install(  # 安装配置与启动目录
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}   # 安装到 share/<包名>
)

install(  # 安装可执行脚本
  PROGRAMS
    scripts/m20_basic_server_backend           # basic_server 后端入口脚本
    scripts/m20_direct_ros_backend             # direct_ros 后端入口脚本
    scripts/m20_locomotion_manager             # 运动管理器入口脚本
    scripts/m20_navigation_adapter             # 导航适配器入口脚本
    scripts/m20_trajectory_progress_tracker    # 轨迹进度跟踪器入口脚本
  DESTINATION lib/${PROJECT_NAME}              # 安装到 lib/<包名>
)

install(  # 安装许可证与说明文档
  FILES LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)  # 构建测试时
  find_package(ament_cmake_pytest REQUIRED)  # pytest 测试支持
  find_package(ament_lint_auto REQUIRED)     # 自动 lint 依赖
  ament_lint_auto_find_test_dependencies()   # 自动查找 lint 测试依赖
  ament_add_pytest_test(  # 运动意图策略测试
    test_motion_intent
    test/test_motion_intent.py
  )
  ament_add_pytest_test(  # launch 契约静态测试
    test_launch_contract
    test/test_launch_contract.py
  )
  ament_add_pytest_test(  # 速度反馈测试
    test_velocity_feedback
    test/test_velocity_feedback.py
  )
  ament_add_pytest_test(  # 能力配置测试
    test_capability_profile
    test/test_capability_profile.py
  )
  ament_add_pytest_test(  # basic_server 协议测试
    test_basic_server_protocol
    test/test_basic_server_protocol.py
  )
  ament_add_pytest_test(  # direct_ros 转移策略测试
    test_direct_ros_policy
    test/test_direct_ros_policy.py
  )
  ament_add_pytest_test(  # 轨迹进度测试
    test_trajectory_progress
    test/test_trajectory_progress.py
  )
endif()

ament_package()  # 生成包（导出构建信息）
