# Copyright(c) 2020 livoxtech limited.

cmake_minimum_required(VERSION 3.14)

# This wrapper exists in the Harbor cross-build image but not in native environments.
set(DEFAULT_CROSS_TOOLCHAIN_FILE "/opt/Cross-Compilation/toolchain.cmake")
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND EXISTS "${DEFAULT_CROSS_TOOLCHAIN_FILE}")
  include("${DEFAULT_CROSS_TOOLCHAIN_FILE}")
endif()
unset(DEFAULT_CROSS_TOOLCHAIN_FILE)

project(livox_ros_driver2)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
