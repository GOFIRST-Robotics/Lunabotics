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
find_package(behaviortree_ros2 REQUIRED)
find_package(rovr_interfaces REQUIRED)

# Add server
add_library(action_server SHARED
  src/behavior_control_tree.cpp)

target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(action_server
  ament_index_cpp
  rclcpp 
  rclcpp_action 
  rclcpp_components 
  behaviortree_ros2
  rovr_interfaces
)

rclcpp_components_register_node(action_server 
PLUGIN "BehaviorControlTreeActionServer" 
EXECUTABLE behavior_control_tree)

install(TARGETS
  action_server
  behavior_control_tree
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Other directores
install(DIRECTORY tree/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
