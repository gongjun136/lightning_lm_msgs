set(SHM_IMGAE_BRIDGE_FILES
  shm_image8k_bridge
  shm_image512k_bridge
  shm_image1m_bridge
  shm_image2m_bridge
  shm_image4m_bridge
  shm_image6m_bridge
  shm_image8m_bridge
  shm_image6m_bridge_6ch
)

foreach(SHM_IMGAE_BRIDGE_EXE ${SHM_IMGAE_BRIDGE_FILES})
  add_executable(${SHM_IMGAE_BRIDGE_EXE} ${SHM_IMGAE_BRIDGE_EXE}.cpp)
  target_include_directories(${SHM_IMGAE_BRIDGE_EXE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  ament_target_dependencies(${SHM_IMGAE_BRIDGE_EXE} "rclcpp" "cv_bridge" "sensor_msgs")
  target_link_libraries(${SHM_IMGAE_BRIDGE_EXE}  ${PROJECT_NAME}_image
      opencv_highgui
      ${cpp_typesupport_target}
  )
  install(TARGETS ${SHM_IMGAE_BRIDGE_EXE} DESTINATION lib/${PROJECT_NAME})
endforeach()

# 新增：6路订阅器
add_executable(shm_image6m_subscriber_6ch shm_image6m_subscriber_6ch.cpp)
target_include_directories(shm_image6m_subscriber_6ch PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
ament_target_dependencies(shm_image6m_subscriber_6ch "rclcpp" "cv_bridge" "sensor_msgs")
target_link_libraries(shm_image6m_subscriber_6ch ${PROJECT_NAME}_image
  opencv_imgcodecs opencv_core opencv_imgproc
  ${cpp_typesupport_target}
)
install(TARGETS shm_image6m_subscriber_6ch DESTINATION lib/${PROJECT_NAME})

# 6路消费者参考示例（遥操推流模块使用）
add_executable(shm_image6m_consumer_6ch shm_image6m_consumer_6ch.cpp)
target_include_directories(shm_image6m_consumer_6ch PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
ament_target_dependencies(shm_image6m_consumer_6ch "rclcpp" "sensor_msgs")
target_link_libraries(shm_image6m_consumer_6ch ${PROJECT_NAME}_image
  opencv_imgcodecs opencv_core opencv_imgproc
  ${cpp_typesupport_target}
)
install(TARGETS shm_image6m_consumer_6ch DESTINATION lib/${PROJECT_NAME})

# 6路压缩桥：直接使用 OpenCV (cvtColor + imencode)，不依赖 cv_bridge，
# 避免与 ROS 自带 cv_bridge 链接的不同版本 OpenCV 产生符号冲突导致 setSize 断言
add_executable(shm_image6m_bridge_6ch_compressed shm_image6m_bridge_6ch_compressed.cpp)
target_include_directories(shm_image6m_bridge_6ch_compressed PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
ament_target_dependencies(shm_image6m_bridge_6ch_compressed "rclcpp" "sensor_msgs")
target_link_libraries(shm_image6m_bridge_6ch_compressed ${PROJECT_NAME}_image
  opencv_imgcodecs opencv_imgproc opencv_core
  ${cpp_typesupport_target}
)
install(TARGETS shm_image6m_bridge_6ch_compressed DESTINATION lib/${PROJECT_NAME})

# Six-channel hardware H.265 encoder. libvideoenc.so is loaded at runtime via dlopen.
# video_encode_api.h is vendored under external/ so this package is self-contained
set(VIDEOENC_API_INCLUDE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../external"
  CACHE PATH "Directory containing video_encode_api.h")
if(NOT EXISTS "${VIDEOENC_API_INCLUDE_DIR}/video_encode_api.h")
  message(FATAL_ERROR
    "video_encode_api.h not found under ${VIDEOENC_API_INCLUDE_DIR}")
endif()

add_executable(shm_image6m_h265_encoder_6ch shm_image6m_h265_encoder_6ch.cpp)
target_include_directories(shm_image6m_h265_encoder_6ch PRIVATE
  ${VIDEOENC_API_INCLUDE_DIR}
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
ament_target_dependencies(shm_image6m_h265_encoder_6ch "rclcpp" "std_msgs")
target_link_libraries(shm_image6m_h265_encoder_6ch
  ${cpp_typesupport_target}
  ${CMAKE_DL_LIBS}
)
install(TARGETS shm_image6m_h265_encoder_6ch DESTINATION lib/${PROJECT_NAME})

# Six-channel hardware H.265 encoder — all-fps variant: encodes every input
# frame with the original capture timestamp (no frame-rate grid throttle,
# bounded raw-frame queue instead of single-slot overwrite).
add_executable(shm_image6m_h265_encoder_6ch_allfps shm_image6m_h265_encoder_6ch_allfps.cpp)
target_include_directories(shm_image6m_h265_encoder_6ch_allfps PRIVATE
  ${VIDEOENC_API_INCLUDE_DIR}
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
ament_target_dependencies(shm_image6m_h265_encoder_6ch_allfps "rclcpp" "std_msgs")
target_link_libraries(shm_image6m_h265_encoder_6ch_allfps
  ${cpp_typesupport_target}
  ${CMAKE_DL_LIBS}
)
install(TARGETS shm_image6m_h265_encoder_6ch_allfps DESTINATION lib/${PROJECT_NAME})

# Six-channel offline H.265 decoder. It groups Annex-B chunks by frame_sequence,
# decodes them with FFmpeg and publishes standard sensor_msgs/Image (bgr8).
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
  libavcodec
  libavutil
  libswscale
)

add_executable(h265_decoder_6ch h265_decoder_6ch.cpp)
ament_target_dependencies(h265_decoder_6ch "rclcpp" "sensor_msgs" "std_msgs")
target_link_libraries(h265_decoder_6ch
  ${cpp_typesupport_target}
  PkgConfig::FFMPEG
)
install(TARGETS h265_decoder_6ch DESTINATION lib/${PROJECT_NAME})

# Offline H.265 decoder that reads a rosbag2 bag (db3/mcap) directly via
# rosbag2_cpp, decodes every CompressedVideo stream and writes the decoded
# sensor_msgs/Image (bgr8) messages into a new rosbag2 bag.
add_executable(h265_decoder_readfile h265_decoder_readfile.cpp)
ament_target_dependencies(h265_decoder_readfile
  "rclcpp" "rosbag2_cpp" "rosbag2_storage" "sensor_msgs" "std_msgs")
target_link_libraries(h265_decoder_readfile
  ${cpp_typesupport_target}
  PkgConfig::FFMPEG
)
install(TARGETS h265_decoder_readfile DESTINATION lib/${PROJECT_NAME})

# Offline bag extractor: reads a rosbag2 bag (db3/mcap) directly and dumps
# perception_swc obstacle inputs (4 fisheye Image6m -> BGR images, and
# /LidarDataInv PointCloud2 -> binary PCD), timestamp-named. No `ros2 bag play`.
add_executable(bag_to_obstacle_input bag_to_obstacle_input.cpp)
target_include_directories(bag_to_obstacle_input PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
ament_target_dependencies(bag_to_obstacle_input
  "rclcpp" "rosbag2_cpp" "rosbag2_storage" "sensor_msgs")
target_link_libraries(bag_to_obstacle_input
  ${cpp_typesupport_target}
  opencv_imgcodecs opencv_imgproc opencv_core
)
install(TARGETS bag_to_obstacle_input DESTINATION lib/${PROJECT_NAME})

# Merge LiDAR PointCloud2 with H.265 CompressedVideo camera streams: decodes
# every H.265 stream to BGR, matches camera frames to each LiDAR frame by
# timestamp, and writes aligned clips (<out>/<bag>_clip/frame_<lidar_ts>/).
add_executable(mergelidar_h265_clip mergelidar_h265_clip.cpp)
target_include_directories(mergelidar_h265_clip PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
ament_target_dependencies(mergelidar_h265_clip
  "rclcpp" "rosbag2_cpp" "rosbag2_storage" "sensor_msgs")
target_link_libraries(mergelidar_h265_clip
  ${cpp_typesupport_target}
  opencv_imgcodecs opencv_imgproc opencv_core
  PkgConfig::FFMPEG
)
install(TARGETS mergelidar_h265_clip DESTINATION lib/${PROJECT_NAME})
