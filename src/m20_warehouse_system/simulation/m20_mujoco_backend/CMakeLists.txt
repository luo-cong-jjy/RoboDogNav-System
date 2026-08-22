# ============================================================
# 文件：CMakeLists.txt
# 用途：m20_mujoco_backend 的 ament_cmake 构建配置。
#       - 安装 Python 包（m20_mujoco_backend/ 目录下的节点模块）；
#       - 安装 config / launch / models 共享目录与脚本、许可文档；
#       - 注册 3 个 pytest 测试（世界生成、后端数学、接口契约），
#         仅在 BUILD_TESTING 开启时执行。
# ============================================================

cmake_minimum_required(VERSION 3.8)
project(m20_mujoco_backend)

# ROS2 ament 基础依赖（cmake + python 包安装支持）。
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装 Python 包：m20_mujoco_backend（backend_node / viewer_node /
# world_generator / dynamics 等模块）。
ament_python_install_package(${PROJECT_NAME})

# 安装配置、launch 与模型模板到 share 目录。
install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行脚本（世界生成 / 后端 / 查看器入口）。
install(
  PROGRAMS
    scripts/m20_generate_mujoco_world
    scripts/m20_mujoco_backend
    scripts/m20_mujoco_viewer
  DESTINATION lib/${PROJECT_NAME}
)

# 安装许可与文档。
install(
  FILES LICENSE README.md THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

# ---------- 测试 ----------
if(BUILD_TESTING)
  # pytest 与 lint 依赖。
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # 世界生成确定性测试（依赖 mujoco 与下游包数据）。
  ament_add_pytest_test(
    test_world_generator
    test/test_world_generator.py
  )
  # 后端数学（坐标换算 / PD 律 / 接触统计）单元测试。
  ament_add_pytest_test(
    test_backend_math
    test/test_backend_math.py
  )
  # launch/接口契约静态检查。
  ament_add_pytest_test(
    test_launch_contract
    test/test_launch_contract.py
  )
endif()

# 生成 ament 包元数据。
ament_package()
