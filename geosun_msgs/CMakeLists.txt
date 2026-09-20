cmake_minimum_required(VERSION 3.5)
project(geosun_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# custom message and service
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AngleSensor.msg"
  "msg/BrakeFrontBackPressure.msg"
  "msg/BrakeFrontBackValveElec.msg"
  "msg/ControlTorque.msg"
  "msg/CanBreak.msg"
  "msg/CurrentVavle.msg"
  "msg/DriveSteerBrake.msg"
  "msg/GearState.msg"
  "msg/ImuBodyAcce.msg"
  "msg/ImuBodyRoll.msg"
  "msg/ImuFrtAngAcce.msg"
  "msg/ImuRawvYaw.msg"
  "msg/ImuRawYaw.msg"
  "msg/ImuRealAngle.msg" 
  "msg/ImuRealPitchV.msg"
  "msg/IVavleFeedback.msg"
  "msg/OrinCanToHMI.msg"
  "msg/ParkBrakeState.msg"
  "msg/PilotPressure.msg"
  "msg/RexrothMBoomBucket.msg"
  "msg/RexrothOther3.msg"
  "msg/SoftStartStopControl.msg"
  "msg/SpeThrCAN4.msg"
  "msg/UJoystick.msg"
  "msg/VCU2HEE.msg"
  "msg/CloudInfo.msg"
  "msg/SlamPose.msg"
  "msg/RtkNav.msg"
  "msg/PosRes.msg"
  "msg/GnssRes.msg"
  "msg/TimeSyn.msg"
  "msg/AbsLidRes.msg"
  "msg/WsRes.msg"
  "msg/RelocatePos.msg"
  "srv/SaveMap.srv"
  "srv/GetInitPosition.srv"
  DEPENDENCIES std_msgs sensor_msgs
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
