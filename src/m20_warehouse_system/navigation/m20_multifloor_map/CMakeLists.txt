cmake_minimum_required(VERSION 3.8)
project(m20_multifloor_map)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/m20_dynamic_local_sensing
    scripts/m20_vendor_sensing_state
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
    test_point_cloud_filter
    test/test_point_cloud_filter.py
  )
endif()

ament_package()
