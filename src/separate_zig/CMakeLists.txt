cmake_minimum_required(VERSION 3.8)
project(ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies natively
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rovr_interfaces REQUIRED)

# Compile your C++ file into a static library
add_library(ros_bridge STATIC src/interface.cpp)

# Link the ROS 2 dependencies
ament_target_dependencies(ros_bridge rclcpp std_msgs rovr_interfaces)
