cmake_minimum_required(VERSION 3.8)
project(m20_industrial_inspection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(motion_adapter_node src/motion_adapter_node.cpp)
ament_target_dependencies(motion_adapter_node rclcpp geometry_msgs)

add_executable(cmd_vel_safety_mux_node src/cmd_vel_safety_mux_node.cpp)
ament_target_dependencies(cmd_vel_safety_mux_node rclcpp geometry_msgs std_msgs)

add_executable(patrol_manager_node src/patrol_manager_node.cpp)
ament_target_dependencies(patrol_manager_node rclcpp geometry_msgs)

add_executable(inspection_mission_node src/inspection_mission_node.cpp)
ament_target_dependencies(
  inspection_mission_node
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
)

add_executable(odom_waypoint_patrol_node src/odom_waypoint_patrol_node.cpp)
ament_target_dependencies(
  odom_waypoint_patrol_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS
  motion_adapter_node
  cmd_vel_safety_mux_node
  patrol_manager_node
  inspection_mission_node
  odom_waypoint_patrol_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/m20_factory_simulation
  scripts/m20_joint_state_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  docs
  launch
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
