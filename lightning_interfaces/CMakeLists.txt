cmake_minimum_required(VERSION 3.16)

project(lightning)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_monitor_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/DebugMessage.msg"
        "msg/FaultStatus.msg"
        "msg/LocalizationStatus.msg"
        "msg/PipelineDiagnostics.msg"
        "msg/VehiclePose.msg"
        "srv/LocCmd.srv"
        "srv/SaveMap.srv"
        DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs diagnostic_monitor_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
