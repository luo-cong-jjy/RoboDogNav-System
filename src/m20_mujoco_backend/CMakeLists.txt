cmake_minimum_required(VERSION 3.8)
project(m20_mujoco_backend)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/m20_generate_mujoco_world
    scripts/m20_mujoco_backend
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES LICENSE README.md THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_world_generator
    test/test_world_generator.py
  )
  ament_add_pytest_test(
    test_backend_math
    test/test_backend_math.py
  )
  ament_add_pytest_test(
    test_launch_contract
    test/test_launch_contract.py
  )
endif()

ament_package()
