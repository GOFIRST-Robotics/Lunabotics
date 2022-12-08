cmake_minimum_required(VERSION 3.8)
project(navx_cpp_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(navx_cpp_node src/navx_cpp_node.cpp)
ament_target_dependencies(navx_cpp_node rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(navx_subscriber src/navx_subscriber.cpp)
ament_target_dependencies(navx_subscriber rclcpp std_msgs sensor_msgs geometry_msgs)

target_include_directories(navx_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(navx_cpp_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17a

install(TARGETS navx_cpp_node navx_subscriber
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()