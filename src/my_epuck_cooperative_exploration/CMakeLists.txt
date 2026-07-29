cmake_minimum_required(VERSION 3.8)
project(my_epuck_cooperative_exploration)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_epuck_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

add_library(cooperative_protocol src/protocol.cpp)
target_include_directories(cooperative_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cooperative_protocol
  geometry_msgs my_epuck_interfaces unique_identifier_msgs)

add_library(cooperative_coordinator src/cooperative_frontier_coordinator.cpp)
target_include_directories(cooperative_coordinator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cooperative_coordinator cooperative_protocol)
ament_target_dependencies(cooperative_coordinator
  my_epuck_interfaces nav2_msgs rclcpp rclcpp_action)

add_executable(cooperative_frontier_coordinator
  src/cooperative_frontier_coordinator_main.cpp)
target_link_libraries(cooperative_frontier_coordinator cooperative_coordinator)
ament_target_dependencies(cooperative_frontier_coordinator rclcpp)

install(TARGETS cooperative_protocol cooperative_coordinator cooperative_frontier_coordinator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE include)
  target_link_libraries(test_protocol cooperative_protocol)
  ament_target_dependencies(test_protocol
    geometry_msgs my_epuck_interfaces unique_identifier_msgs)
  ament_add_gtest(test_native_harness test/test_native_harness.cpp)
  target_link_libraries(test_native_harness cooperative_protocol cooperative_coordinator)
  ament_target_dependencies(test_native_harness my_epuck_interfaces nav2_msgs rclcpp rclcpp_action)
endif()

ament_export_include_directories(include)
ament_export_libraries(cooperative_protocol)
ament_export_dependencies(geometry_msgs my_epuck_interfaces unique_identifier_msgs)
ament_package()
