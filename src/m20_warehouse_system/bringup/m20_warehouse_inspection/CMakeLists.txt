cmake_minimum_required(VERSION 3.8)  # 声明最低 CMake 版本要求（3.8）
project(m20_warehouse_inspection)  # 工程名：m20_warehouse_inspection（必须与包名/package.xml 一致）

# —— 查找依赖包（全部 REQUIRED：缺失任一包即编译失败）——
find_package(ament_cmake REQUIRED)  # ament 构建系统核心（提供 install、ament_package 等）
find_package(ament_cmake_python REQUIRED)  # ament Python 支持（提供 ament_python_install_package 等）

# —— 安装 Python 包源码 ——
ament_python_install_package(${PROJECT_NAME})  # 安装 m20_warehouse_inspection 目录下的 Python 包（可被 import）

# —— 安装资源目录 ——
install(  # 安装 launch/config/docs/maps/patches/rviz/tools 目录
  DIRECTORY config docs launch maps patches rviz tools
  DESTINATION share/${PROJECT_NAME}  # 安装到 share/m20_warehouse_inspection（运行时经 get_package_share_directory 查找）
)

# —— 安装 Python 脚本（安装时自动赋予可执行权限）——
install(
  PROGRAMS
    scripts/m20_clearance_probe  # 间隙探测脚本
    scripts/m20_command_envelope_probe  # 命令包络探测脚本
    scripts/m20_flat_map_server  # 平面多楼层地图服务器脚本
    scripts/m20_generate_maps  # 地图生成脚本
    scripts/m20_hardware_localization_adapter  # 硬件定位适配器脚本
    scripts/m20_import_site_pcd  # 站点 PCD 导入脚本
    scripts/m20_phase4_acceptance  # 阶段 4 验收脚本
    scripts/m20_phase5_regression  # 阶段 5 回归脚本
    scripts/m20_start_inspection  # 巡检启动脚本
    scripts/m20_validate_config  # 配置校验脚本
  DESTINATION lib/${PROJECT_NAME}  # 安装到 lib/m20_warehouse_inspection
)

# —— 安装单个工具脚本并重命名 ——
install(  # 安装导航运动探针工具
  PROGRAMS tools/navigation_motion_probe.py
  DESTINATION lib/${PROJECT_NAME}  # 安装到 lib/m20_warehouse_inspection
  RENAME m20_navigation_motion_probe  # 重命名为无 .py 后缀的可执行名
)

# —— 安装文档与依赖清单文件 ——
install(  # 安装说明文档/许可证/第三方声明/依赖清单
  FILES
    CHANGELOG.md  # 变更日志
    LICENSE  # 许可证
    README.md  # 说明文档
    THIRD_PARTY_NOTICES.md  # 第三方声明
    dependencies.repos  # 主依赖清单（vcs 格式）
    dependencies_sdk.repos  # SDK 依赖清单
  DESTINATION share/${PROJECT_NAME}  # 安装到 share/m20_warehouse_inspection
)

# —— 测试段（仅 BUILD_TESTING 开启时）——
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # ament 自动 lint 框架
  ament_lint_auto_find_test_dependencies()  # 自动查找并注册 lint 测试依赖

  find_package(ament_cmake_pytest REQUIRED)  # pytest 测试支持
  ament_add_pytest_test(  # 配置校验测试
    test_configuration
    test/test_configuration.py
  )
  ament_add_pytest_test(  # 地图资源测试
    test_map_assets
    test/test_map_assets.py
  )
  ament_add_pytest_test(  # 阶段 2 bringup 测试
    test_phase2_bringup
    test/test_phase2_bringup.py
  )
  ament_add_pytest_test(  # 阶段 3 bringup 测试
    test_phase3_bringup
    test/test_phase3_bringup.py
  )
  ament_add_pytest_test(  # 阶段 4 bringup 测试
    test_phase4_bringup
    test/test_phase4_bringup.py
  )
  ament_add_pytest_test(  # 回归策略测试
    test_regression_policy
    test/test_regression_policy.py
  )
  ament_add_pytest_test(  # 阶段 5 工作区测试
    test_phase5_workspace
    test/test_phase5_workspace.py
  )
  ament_add_pytest_test(  # 阶段 5 bringup 测试
    test_phase5_bringup
    test/test_phase5_bringup.py
  )
  ament_add_pytest_test(  # 障碍间距扫描测试
    test_spacing_sweep
    test/test_spacing_sweep.py
  )
  ament_add_pytest_test(  # MuJoCo bringup 测试
    test_mujoco_bringup
    test/test_mujoco_bringup.py
  )
  ament_add_pytest_test(  # 窄通道测试
    test_narrow_passage
    test/test_narrow_passage.py
  )
  ament_add_pytest_test(  # 硬件 bringup 测试
    test_hardware_bringup
    test/test_hardware_bringup.py
  )
  ament_add_pytest_test(  # 硬件定位测试
    test_hardware_localization
    test/test_hardware_localization.py
  )
endif()

ament_package()  # 生成 ament 包元数据：导出 share 目录、生成包索引（必须在所有 install 之后调用）
