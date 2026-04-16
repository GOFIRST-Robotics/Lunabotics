cmake_minimum_required(VERSION 3.8)
project(behavior_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rovr_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add the Standalone Executable
add_executable(behavior_control_node src/behavior_control_server.cpp)

target_include_directories(behavior_control_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(behavior_control_node
  ament_index_cpp
  rclcpp 
  rclcpp_action 
  rclcpp_components 
  behaviortree_cpp
  behaviortree_ros2
  rovr_interfaces
  nav2_msgs
  geometry_msgs
  sensor_msgs
)

# Install the node
install(TARGETS
  behavior_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the tree files into a 'tree' subdirectory in share
install(DIRECTORY tree/
  DESTINATION share/${PROJECT_NAME}/tree
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
