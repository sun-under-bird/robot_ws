cmake_minimum_required(VERSION 3.8)
project(stereo_camera_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED)


add_executable(stereo_camera_node src/stereo_camera_node.cpp)
add_executable(left_camera_node src/left_camera_node.cpp)
add_executable(right_camera_node src/right_camera_node.cpp)
add_executable(camera_info src/camera_info.cpp)
add_executable(stereo_split_node src/stereo_split_node.cpp)
add_executable(stereo_split_node_rgb src/stereo_split_node_rgb.cpp)
add_executable(d435i_kalibr_camera_info_node src/d435i_kalibr_camera_info_node.cpp)

target_include_directories(stereo_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(stereo_camera_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  stereo_camera_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "message_filters"
)
ament_target_dependencies(
  left_camera_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
)
ament_target_dependencies(
  right_camera_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
)

ament_target_dependencies(
  camera_info
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "message_filters"
  "ament_index_cpp"
)
ament_target_dependencies(
  stereo_split_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "message_filters"
)
ament_target_dependencies(
  stereo_split_node_rgb
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "message_filters"
)
ament_target_dependencies(
  d435i_kalibr_camera_info_node
  "rclcpp"
  "sensor_msgs"
  "ament_index_cpp"
)

target_link_libraries(
  stereo_camera_node
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(
  left_camera_node
  ${OpenCV_LIBRARIES}
)
target_link_libraries(
  right_camera_node
  ${OpenCV_LIBRARIES}
)
target_link_libraries(
  camera_info
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
target_link_libraries(
  stereo_split_node
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
target_link_libraries(
  stereo_split_node_rgb
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
target_link_libraries(
  d435i_kalibr_camera_info_node
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS 
  stereo_camera_node
  left_camera_node
  right_camera_node
  camera_info
  stereo_split_node
  stereo_split_node_rgb
  d435i_kalibr_camera_info_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 保留 launch 目录层级，使 get_package_share_directory(...)/launch 可正常定位。
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
