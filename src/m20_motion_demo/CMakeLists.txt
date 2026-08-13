cmake_minimum_required(VERSION 3.8)
project(m20_motion_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(line_demo src/line_demo.cpp)
ament_target_dependencies(line_demo rclcpp geometry_msgs)

add_executable(circle_demo src/circle_demo.cpp)
ament_target_dependencies(circle_demo rclcpp geometry_msgs)

install(TARGETS
  line_demo
  circle_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
