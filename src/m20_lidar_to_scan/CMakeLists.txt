cmake_minimum_required(VERSION 3.8)
project(m20_lidar_to_scan)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  docs
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/compute_path_probe
  scripts/grid_astar_planner
  scripts/lifecycle_configure_activate
  scripts/pcd_to_occupancy_grid
  scripts/pcd_slice_publisher
  scripts/scan_probe
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
