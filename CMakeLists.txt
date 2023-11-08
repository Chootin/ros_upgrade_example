cmake_minimum_required(VERSION 3.5)
project(ros_upgrade_example)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

## Generate actions, messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExampleMsg.msg"
  DEPENDENCIES
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS ${PROJECT_NAME}/upgrade_example.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)

ament_package()