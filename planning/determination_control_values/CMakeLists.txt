cmake_minimum_required(VERSION 3.14)
project(determination_control_values)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(determination_control_values SHARED
  src/determination_control_values_node.cpp
)

target_link_libraries(determination_control_values)

rclcpp_components_register_node(determination_control_values
  PLUGIN "motion_planning::DeterminationControlValues"
  EXECUTABLE determination_control_values_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)