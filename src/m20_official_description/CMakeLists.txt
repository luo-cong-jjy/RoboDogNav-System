cmake_minimum_required(VERSION 3.8)
project(m20_official_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf xacro
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    LICENSE
    README.md
    SOURCE_MODEL.json
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_model_assets
    test/test_model_assets.py
  )
endif()

ament_package()
