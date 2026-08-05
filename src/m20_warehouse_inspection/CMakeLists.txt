cmake_minimum_required(VERSION 3.8)
project(m20_warehouse_inspection)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config docs launch maps patches rviz tools
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/m20_boundary_recovery_probe
    scripts/m20_clearance_probe
    scripts/m20_command_envelope_probe
    scripts/m20_flat_map_server
    scripts/m20_generate_maps
    scripts/m20_phase4_acceptance
    scripts/m20_phase5_regression
    scripts/m20_start_inspection
    scripts/m20_validate_config
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    CHANGELOG.md
    LICENSE
    README.md
    THIRD_PARTY_NOTICES.md
    dependencies.repos
    dependencies_sdk.repos
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_configuration
    test/test_configuration.py
  )
  ament_add_pytest_test(
    test_map_assets
    test/test_map_assets.py
  )
  ament_add_pytest_test(
    test_phase2_bringup
    test/test_phase2_bringup.py
  )
  ament_add_pytest_test(
    test_phase3_bringup
    test/test_phase3_bringup.py
  )
  ament_add_pytest_test(
    test_phase4_bringup
    test/test_phase4_bringup.py
  )
  ament_add_pytest_test(
    test_regression_policy
    test/test_regression_policy.py
  )
  ament_add_pytest_test(
    test_phase5_workspace
    test/test_phase5_workspace.py
  )
  ament_add_pytest_test(
    test_phase5_bringup
    test/test_phase5_bringup.py
  )
  ament_add_pytest_test(
    test_spacing_sweep
    test/test_spacing_sweep.py
  )
  ament_add_pytest_test(
    test_mujoco_bringup
    test/test_mujoco_bringup.py
  )
  ament_add_pytest_test(
    test_boundary_recovery
    test/test_boundary_recovery.py
  )
  ament_add_pytest_test(
    test_narrow_passage
    test/test_narrow_passage.py
  )
endif()

ament_package()
