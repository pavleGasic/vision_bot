cmake_minimum_required(VERSION 3.8)
project(visionbot_benchmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(benchmark src/benchmark_runner.cpp)
ament_target_dependencies(benchmark rclcpp rclcpp_action nav2_msgs geometry_msgs)

INSTALL(
  DIRECTORY include
  DESTINATION include
)

# install(
# DIRECTORY
# config
# launch
# DESTINATION share/${PROJECT_NAME}/
# )
install(
  TARGETS
  benchmark
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
