cmake_minimum_required(VERSION 3.8)
project(odom_covariance_relay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(odom_covariance_relay_node src/odom_covariance_relay.cpp)
target_compile_features(odom_covariance_relay_node PUBLIC cxx_std_17)
ament_target_dependencies(odom_covariance_relay_node nav_msgs rclcpp)

install(TARGETS odom_covariance_relay_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
