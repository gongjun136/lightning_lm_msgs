add_library(${PROJECT_NAME}_image
    ./opencv_conversions.cpp
    ./rgb_colors.cpp
)
target_include_directories(${PROJECT_NAME}_image PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )
target_link_libraries(${PROJECT_NAME}_image
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
)
target_link_libraries(${PROJECT_NAME}_image
    rcpputils::rcpputils
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_image
  ${cpp_typesupport_target}
  )
ament_target_dependencies(${PROJECT_NAME}_image
  "rclcpp"
  "std_msgs"
)

install(TARGETS ${PROJECT_NAME}_image
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/
  )
ament_export_libraries(${PROJECT_NAME}_image)
