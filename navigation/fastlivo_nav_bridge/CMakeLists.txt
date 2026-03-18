cmake_minimum_required(VERSION 3.8)
project(fastlivo_nav_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fastlivo_nav_bridge_node src/fastlivo_nav_bridge_node.cpp)
target_include_directories(fastlivo_nav_bridge_node
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(fastlivo_nav_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros)

install(TARGETS
  fastlivo_nav_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
