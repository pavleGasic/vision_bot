cmake_minimum_required(VERSION 3.8)
project(vision_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(heartbeat src/heartbeat/heartbeat.cpp)
add_executable(navigation src/navigation/navigation_node.cpp)
ament_target_dependencies(heartbeat rclcpp std_msgs)
ament_target_dependencies(navigation rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs vision_msgs)

install(TARGETS
  heartbeat
  navigation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  description
  config
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(perception
  PACKAGE_DIR src/perception
)

install(PROGRAMS
  src/perception/yolo_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
