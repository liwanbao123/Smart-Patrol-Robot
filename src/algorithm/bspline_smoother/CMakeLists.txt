cmake_minimum_required(VERSION 3.5)
project(bspline_smoother)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

nav2_package()

include_directories(
  include
)

set(dependencies
  angles
  rclcpp
  rclcpp_components
  rclcpp_action
  rclcpp_components
  std_msgs
  nav2_msgs
  nav_2d_utils
  nav_2d_msgs
  nav2_util
  nav2_core
  pluginlib
)

# Simple Smoother plugin
add_library(bspline_smoother SHARED
  src/bspline_smoother.cpp
  src/bspline_curve.cpp
  src/curve.cpp
  src/math_helper.cpp
)
ament_target_dependencies(bspline_smoother
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core bspline_smoother.xml)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

install(
  TARGETS ${project_name} bspline_smoother
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(${project_name} bspline_smoother)
ament_export_dependencies(${dependencies})
ament_package()
