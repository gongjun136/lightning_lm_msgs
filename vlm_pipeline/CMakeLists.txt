cmake_minimum_required(VERSION 3.8)
project(vlm_pipeline)

# Message-only subset -- see package.xml for provenance.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehiclePose.msg"
  "msg/TaskItem.msg"
  "msg/TaskQueueRequest.msg"
  "msg/TaskQueueResponse.msg"
  "msg/PhaseTransition.msg"
  "msg/VehicleState.msg"
  "msg/HmiDisplay.msg"
  "msg/VlmError.msg"
  "msg/WholeCycleSummary.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
