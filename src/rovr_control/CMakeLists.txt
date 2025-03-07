cmake_minimum_required(VERSION 3.8)
project(rovr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rovr_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(behavior_tree_executor rovr_control/behavior_tree_executor.cpp)
ament_target_dependencies(behavior_tree_executor rclcpp std_srvs behaviortree_ros2 behaviortree_cpp rovr_interfaces)

add_executable(calibrate_field_coordinate action/calibrate_field_coordinate.cpp)
ament_target_dependencies(calibrate_field_coordinate rclcpp std_srvs behaviortree_ros2 behaviortree_cpp rovr_interfaces)

target_include_directories(behavior_tree_executor PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(calibrate_field_coordinate PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_features(behavior_tree_executor PUBLIC c_std_99 cxx_std_17)
target_compile_features(calibrate_field_coordinate PUBLIC c_std_99 cxx_std_17)

install(TARGETS behavior_tree_executor calibrate_field_coordinate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch action
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif() 

ament_package()