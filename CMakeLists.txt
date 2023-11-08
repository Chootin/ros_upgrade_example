cmake_minimum_required(VERSION 3.0.2)
project(ros_upgrade_example)

find_package(catkin REQUIRED COMPONENTS
find_package(message_generation)

find_package(rospy REQUIRED)
find_package(std_msgs REQUIRED)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ExampleMsg.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_upgrade_example
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)