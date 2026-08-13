cmake_minimum_required(VERSION 3.5)
project(m20_foxy_nav_deploy)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  docs
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
