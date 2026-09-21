cmake_minimum_required(VERSION 3.5)

project(shm_msgs)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# add installation directory here
# list(INSERT CMAKE_PREFIX_PATH 0 /opt/open3d/open3d)
list(INSERT CMAKE_PREFIX_PATH 0 /opt/opencv/opencv-4.6.0/)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3 REQUIRED)
endif()

set(dependencies
  rclcpp
  rosidl_default_runtime
  cv_bridge
  sensor_msgs
  std_msgs
)

set(msg_files
  "msg/String.msg"
  "msg/Header.msg"
  "msg/EulerAngle.msg"
  "msg/SensorID.msg"
  "msg/Image8k.msg"
  "msg/Image512k.msg"
  "msg/Image1m.msg"
  "msg/Image2m.msg"
  "msg/Image4m.msg"
  "msg/Image6m.msg"
  "msg/Image8m.msg"
  "msg/CompressedVideo.msg"
  "msg/CompressedImageFrame.msg"
  "msg/ImageFrame.msg"
  "msg/PointCloudData.msg"
  "msg/LidarFrame.msg"
  "msg/LocalizationInfo.msg"
  "msg/PolyLine.msg"
  "msg/Polygon.msg"
  "msg/FreeSpace.msg"
  "msg/Curb.msg"
  "msg/Occupancy.msg"
  "msg/DrivableArea.msg"
  "msg/DynamicObject.msg"
  "msg/StaticObject.msg"
  "msg/Obstacle.msg"
  "msg/PalletData.msg"
  "msg/Pallet.msg"
  "msg/SlotData.msg"
  "msg/Slot.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

if(cpp_typesupport_target)
  add_library(${PROJECT_NAME}_library INTERFACE)
  target_include_directories(${PROJECT_NAME}_library INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${PROJECT_NAME}_library INTERFACE
  "${cpp_typesupport_target}")

  install(
    TARGETS ${PROJECT_NAME}_library EXPORT export_${PROJECT_NAME}
  )
endif()

include_directories(
  include
)

add_subdirectory(lib)

add_subdirectory(tool)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS scripts/extract_h265_from_bag.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
