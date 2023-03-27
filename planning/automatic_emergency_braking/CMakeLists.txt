cmake_minimum_required(VERSION 3.14)
project(automatic_emergency_braking)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(automatic_emergency_braking SHARED
  src/automatic_emergency_braking_node.cpp
)

target_link_libraries(automatic_emergency_braking)

rclcpp_components_register_node(automatic_emergency_braking
  PLUGIN "motion_planning::AutomaticEmergencyBrakingNode"
  EXECUTABLE automatic_emergency_braking_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)