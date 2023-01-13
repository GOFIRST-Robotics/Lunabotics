cmake_minimum_required(VERSION 3.5)

project(ros2socketcan_bridge)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(can_msgs REQUIRED)

include_directories(include)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin ) 
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib ) 
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )

add_executable(ros2can_bridge src/ros2socketcan.cpp)

TARGET_LINK_LIBRARIES(ros2can_bridge dl -lboost_system -pthread  -lcrypt -lm -lpthread)

ament_target_dependencies(ros2can_bridge rclcpp std_msgs can_msgs)
install(TARGETS 
        ros2can_bridge
        DESTINATION bin)
        
ament_package()