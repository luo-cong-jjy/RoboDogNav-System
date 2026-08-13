cmake_minimum_required(VERSION 3.8)
project(m20_industrial_inspection_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(gazebo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(m20_sinusoidal_obstacle_plugin SHARED
  src/sinusoidal_obstacle_plugin.cpp
)
target_include_directories(m20_sinusoidal_obstacle_plugin
  PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(m20_sinusoidal_obstacle_plugin
  ${GAZEBO_LIBRARIES}
)

add_library(m20_waypoint_obstacle_plugin SHARED
  src/waypoint_obstacle_plugin.cpp
)
target_include_directories(m20_waypoint_obstacle_plugin
  PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(m20_waypoint_obstacle_plugin
  ${GAZEBO_LIBRARIES}
)

add_library(m20_four_wheel_drive_plugin SHARED
  src/four_wheel_drive_plugin.cpp
)
target_include_directories(m20_four_wheel_drive_plugin
  PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
)
ament_target_dependencies(m20_four_wheel_drive_plugin
  builtin_interfaces
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)
target_link_libraries(m20_four_wheel_drive_plugin
  ${GAZEBO_LIBRARIES}
)

add_library(m20_legged_kinematic_plugin SHARED
  src/legged_kinematic_plugin.cpp
)
target_include_directories(m20_legged_kinematic_plugin
  PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
)
ament_target_dependencies(m20_legged_kinematic_plugin
  builtin_interfaces
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)
target_link_libraries(m20_legged_kinematic_plugin
  ${GAZEBO_LIBRARIES}
)

install(TARGETS
  m20_sinusoidal_obstacle_plugin
  m20_waypoint_obstacle_plugin
  m20_four_wheel_drive_plugin
  m20_legged_kinematic_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  config
  docs
  launch
  maps
  models
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  model.config
  model.sdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/factory_inspection_nav2_mission
  scripts/goal_pose_restamper
  scripts/keyboard_cmd_vel_teleop
  scripts/m20_standing_joint_state_publisher
  scripts/odom_waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
