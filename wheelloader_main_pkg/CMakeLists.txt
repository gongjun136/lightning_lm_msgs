cmake_minimum_required(VERSION 3.5)
project(wheelloader_main_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找核心依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# 设置消息文件
set(msg_files
  "msg/BodyStateFeed.msg"
  "msg/DownBodyParamSetting.msg"
  "msg/UpBodyParamSetting.msg"
  "msg/CUData.msg"
  "msg/RTKmsg.msg"
  "msg/AutonomousTaskProcess.msg"
  "msg/AutonomousTruckPosition.msg"
  "msg/AutonomousTruckWarning.msg"
  "msg/TaskDo.msg"
  "msg/Undet.msg"
  "msg/PlanPath.msg"
  "msg/MultiArray300.msg"
  "msg/MultiArray70000.msg"
  "msg/MultiArray8e6.msg"
  "msg/Int8Array8000000.msg"
  "msg/AutoParking.msg"
  "msg/ChooseKind.msg"
  "msg/ChooseMission.msg"
  "msg/EmergStop.msg"
  "msg/MasterMpcSwtich.msg"
  "msg/TeleElec.msg"
  "msg/Status.msg"
  "msg/MaterialMapping.msg"
  "msg/MaterialMap.msg"
  "msg/Warning.msg"
  "msg/MaterialLevel.msg"
  "msg/Material.msg"
  "msg/Dec2conLowerPoint.msg"
  "msg/Dec2conLowerPoints.msg"
  "msg/PlanResult.msg"
  "msg/Dec2conLowerPath.msg"
  "msg/Dec2conUpperCtrl.msg"
  "msg/Dec2conLowerTurn.msg"
  "msg/Con2decLowerState.msg"
  "msg/Dec2planGlobalTask.msg"
  "msg/Dec2planLocalTask.msg"
  "msg/Dec2planLocalFeeds.msg"
  "msg/Dec2planGeneralParams.msg"
)

# 设置服务文件
set(srv_files
  "srv/BodyState.srv"
  "srv/MsgOperation.srv"
  "srv/AutoParking.srv"
  "srv/ChooseKind.srv"
  "srv/ChooseMission.srv"
  "srv/EmergStop.srv"
  "srv/StartMining.srv"
  "srv/TeleElec.srv"
  "srv/DecisionNodeError.srv"
  "srv/ErrorY.srv"
  "srv/MaterialLevel.srv"
)

# 生成消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces 
)

# 安装消息
install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)
install(
  DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)
# 导出包含路径
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
