cmake_minimum_required(VERSION 3.8)
project(visionbot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(a_star_planner_node SHARED src/a_star_planner_node.cpp src/a_star_core.cpp)
ament_target_dependencies(a_star_planner_node rclcpp rclcpp_action nav_msgs geometry_msgs tf2_ros nav2_core nav2_costmap_2d nav2_util pluginlib nav2_msgs)

pluginlib_export_plugin_description_file(nav2_core global_planner_plugins.xml)

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
  motion_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  a_star_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES global_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs nav2_core nav2_costmap_2d nav2_util pluginlib)
ament_package()
