cmake_minimum_required(VERSION 3.8)
project(m20_lidar_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud_roi_bridge src/pointcloud_roi_bridge.cpp)
ament_target_dependencies(pointcloud_roi_bridge
  rclcpp
  sensor_msgs
)

install(TARGETS pointcloud_roi_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/probe_rslidar_points
  scripts/run_backpack_rshelios
  scripts/run_backpack_m20_rs48
  scripts/run_gos_rshelios
  scripts/run_gos_m20_rs48
  scripts/tcpdump_backpack
  scripts/tcpdump_gos
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  dds
  docs
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
