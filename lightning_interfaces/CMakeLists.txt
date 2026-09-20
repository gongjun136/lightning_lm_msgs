cmake_minimum_required(VERSION 3.16)

include(/opt/Cross-Compilation/toolchain.cmake)
project(lightning)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "${CMAKE_SOURCE_DIR}:msg/DebugMessage.msg"
        "${CMAKE_SOURCE_DIR}:msg/FaultStatus.msg"
        "${CMAKE_SOURCE_DIR}:msg/LocalizationStatus.msg"
        "${CMAKE_SOURCE_DIR}:msg/PipelineDiagnostics.msg"
        "${CMAKE_SOURCE_DIR}:msg/VehiclePose.msg"
        "${CMAKE_SOURCE_DIR}:srv/LocCmd.srv"
        "${CMAKE_SOURCE_DIR}:srv/SaveMap.srv"
        DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

#file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/lightning" CONTENT "")
#install(FILES "${CMAKE_CURRENT_BINARY_DIR}/lightning"
#        DESTINATION share/ament_index/resource_index/packages)
#install(FILES package.xml
#        DESTINATION share/${PROJECT_NAME})
