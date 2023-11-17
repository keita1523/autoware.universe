cmake_minimum_required(VERSION 3.5)
project(livox_pointcloud)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  # Ignore PCL errors in Clang
  add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types)
endif()

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(livox_rawdata SHARED
  src/livox_pointcloud.cpp
)

ament_auto_add_executable(livox_cloud_node
  src/livox_pointcloud.cpp
)

rclcpp_components_register_node(livox_rawdata
  PLUGIN "livox_pointcloud::Convert"
  EXECUTABLE cloud_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)
