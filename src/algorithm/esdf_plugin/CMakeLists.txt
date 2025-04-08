cmake_minimum_required(VERSION 3.5)
project(esdf_plugin)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  nav2_costmap_2d
  pluginlib
  nav_msgs
)

add_library(${library_name} SHARED
  src/distance_layer.cpp
  src/esdf.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d distance_layer_costmap_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# message("${TARGETS}")
install(DIRECTORY include/
  DESTINATION include/
)

install(FILES distance_layer_costmap_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
