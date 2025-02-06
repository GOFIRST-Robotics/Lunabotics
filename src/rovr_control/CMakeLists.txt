cmake_minimum_required(VERSION 3.8)
project(rovr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rovr_interfaces REQUIRED)

add_executable(calibrate_field_coordinate action/calibrate_field_coordinate.cpp)
ament_target_dependencies(calibrate_field_coordinate rclcpp std_msgs can_msgs geometry_msgs behaviortree_ros2 rovr_interfaces)

target_include_directories(calibrate_field_coordinate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(calibrate_field_coordinate PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS calibrate_field_coordinate
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()