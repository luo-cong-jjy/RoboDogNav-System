cmake_minimum_required(VERSION 3.8)
project(m20_locomotion_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/m20_basic_server_backend
    scripts/m20_direct_ros_backend
    scripts/m20_locomotion_manager
    scripts/m20_navigation_adapter
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
    test_motion_intent
    test/test_motion_intent.py
  )
  ament_add_pytest_test(
    test_launch_contract
    test/test_launch_contract.py
  )
  ament_add_pytest_test(
    test_velocity_feedback
    test/test_velocity_feedback.py
  )
  ament_add_pytest_test(
    test_capability_profile
    test/test_capability_profile.py
  )
  ament_add_pytest_test(
    test_basic_server_protocol
    test/test_basic_server_protocol.py
  )
  ament_add_pytest_test(
    test_direct_ros_policy
    test/test_direct_ros_policy.py
  )
endif()

ament_package()
