cmake_minimum_required(VERSION 3.8)
project(nav2_esdf)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Timing-critical code: never silently build unoptimised (colcon passes no build type by default).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

set(dependencies nav2_costmap_2d nav_msgs pluginlib rclcpp rclcpp_lifecycle)

add_library(nav2_esdf_layer SHARED src/esdf_layer.cpp)
target_include_directories(nav2_esdf_layer PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(nav2_esdf_layer ${dependencies})

# Standalone algorithm self-test (no ROS): ros2 run nav2_esdf nav2_esdf_selftest
add_executable(nav2_esdf_selftest test/selftest.cpp)
target_include_directories(nav2_esdf_selftest PRIVATE include)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS nav2_esdf_layer
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS nav2_esdf_selftest DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(nav2_esdf_layer)
ament_export_dependencies(${dependencies})
ament_package()
