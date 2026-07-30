cmake_minimum_required(VERSION 3.8)
project(m20_warehouse_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FloorState.msg"
  "msg/LocalSensingState.msg"
  "msg/MissionState.msg"
  "msg/NavigationState.msg"
  "srv/ResetNavigation.srv"
  "srv/ControlMission.srv"
  "srv/SetSimulationPose.srv"
  "srv/SwitchMap.srv"
  "action/NavigateFloor.action"
  "action/RunMission.action"
  "action/SwitchFloor.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
