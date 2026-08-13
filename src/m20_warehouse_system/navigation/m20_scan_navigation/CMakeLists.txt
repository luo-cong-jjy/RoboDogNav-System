cmake_minimum_required(VERSION 3.8)
project(m20_scan_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/m20_clearance_report
    scripts/m20_navigation_gateway
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES LICENSE NOTICE README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_navigation_contract
    test/test_navigation_contract.py
  )
  ament_add_pytest_test(
    test_clearance
    test/test_clearance.py
  )
endif()

ament_package()
