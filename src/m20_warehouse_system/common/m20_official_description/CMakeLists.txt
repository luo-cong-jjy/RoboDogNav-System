cmake_minimum_required(VERSION 3.8)  # 最低 CMake 版本
project(m20_official_description)    # 项目名：Deep Robotics M20 官方模型描述包

find_package(ament_cmake REQUIRED)   # ament 构建系统

# 安装模型/资源目录到包的 share 目录（运行时通过 package share 访问）
install(
  DIRECTORY launch meshes rviz urdf xacro  # 启动文件、网格、rviz 配置、urdf、xacro
  DESTINATION share/${PROJECT_NAME}
)

# 安装文档与许可文件
install(
  FILES
    LICENSE              # BSD-3-Clause 许可证
    README.md            # 说明文档
    SOURCE_MODEL.json    # 源模型元数据（记录官方模型来源信息）
  DESTINATION share/${PROJECT_NAME}
)

# 测试（仅构建时启用 BUILD_TESTING 才编译测试）
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)   # pytest 支持
  find_package(ament_lint_auto REQUIRED)      # 自动 lint 检查
  ament_lint_auto_find_test_dependencies()    # 自动发现 lint 依赖
  ament_add_pytest_test(
    test_model_assets       # 测试名：验证模型资源完整性
    test/test_model_assets.py
  )
endif()

ament_package()  # 标准 ament 打包
