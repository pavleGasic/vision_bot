cmake_minimum_required(VERSION 3.8)
project(visionbot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(a_star_planner_node
  src/a_star_planner_node.cpp
  src/a_star_core.cpp
)
ament_target_dependencies(a_star_planner_node rclcpp nav_msgs geometry_msgs tf2_ros)

add_executable(motion_planner
  src/motion_planner_node.cpp
  src/pure_pursuit.cpp
  src/path_handler.cpp
)
ament_target_dependencies(motion_planner rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  a_star_planner_node
  motion_planner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)
ament_package()
