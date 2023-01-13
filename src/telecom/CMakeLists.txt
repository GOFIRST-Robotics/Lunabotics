cmake_minimum_required(VERSION 3.8)
project(telecom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mc_to_robot_node src/mc_to_robot_node.cpp)
ament_target_dependencies(mc_to_robot_node rclcpp std_msgs sensor_msgs)

add_executable(robot_to_mc_node src/robot_to_mc_node.cpp)
ament_target_dependencies(robot_to_mc_node rclcpp sensor_msgs)

target_include_directories(mc_to_robot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mc_to_robot_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17a

target_include_directories(robot_to_mc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_to_mc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17a

install(TARGETS mc_to_robot_node #robot_to_mc_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()