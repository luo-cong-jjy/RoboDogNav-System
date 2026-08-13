cmake_minimum_required(VERSION 3.8)
project(m20_warehouse_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/m20_rviz_kinematic_backend
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
    test_kinematics
    test/test_kinematics.py
  )
endif()

ament_package()
