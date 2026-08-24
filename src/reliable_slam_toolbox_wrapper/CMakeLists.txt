cmake_minimum_required(VERSION 3.16)
project(reliable_slam_toolbox_wrapper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slam_toolbox REQUIRED)

add_executable(reliable_async_slam_toolbox_node src/reliable_async_slam_toolbox_node.cpp)
ament_target_dependencies(reliable_async_slam_toolbox_node rclcpp slam_toolbox)
target_link_libraries(reliable_async_slam_toolbox_node
  async_slam_toolbox toolbox_common kartoSlamToolbox
  boost_serialization boost_thread tbb)
target_link_directories(reliable_async_slam_toolbox_node PRIVATE /opt/ros/jazzy/lib)

install(TARGETS reliable_async_slam_toolbox_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
