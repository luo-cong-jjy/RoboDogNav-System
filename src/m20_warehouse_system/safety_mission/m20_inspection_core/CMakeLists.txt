cmake_minimum_required(VERSION 3.8)
project(m20_inspection_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/m20_collision_guard
    scripts/m20_floor_switch_manager
    scripts/m20_mission_executor
    scripts/m20_safety_supervisor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_safety_policy
    test/test_safety_policy.py
  )
  ament_add_pytest_test(
    test_collision_policy
    test/test_collision_policy.py
  )
  ament_add_pytest_test(
    test_floor_switch_policy
    test/test_floor_switch_policy.py
  )
  ament_add_pytest_test(
    test_mission_policy
    test/test_mission_policy.py
  )
endif()

ament_package()
