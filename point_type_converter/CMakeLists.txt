cmake_minimum_required(VERSION 3.8)
project(point_type_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(point_type_converter_component SHARED src/point_type_converter.cpp)

target_include_directories(
  point_type_converter_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(point_type_converter_component "rclcpp"
                          "rclcpp_components" "sensor_msgs" "std_msgs")

rclcpp_components_register_node(
  point_type_converter_component PLUGIN
  "autoware_tartan_bridge::PointTypeConverter" EXECUTABLE
  point_type_converter_node)

install(
  TARGETS point_type_converter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
