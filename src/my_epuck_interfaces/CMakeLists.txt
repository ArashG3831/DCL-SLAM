cmake_minimum_required(VERSION 3.8)
project(my_epuck_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PeerMap.msg"
  DEPENDENCIES builtin_interfaces nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
