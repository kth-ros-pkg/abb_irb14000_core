cmake_minimum_required(VERSION 2.8.3)
project(abb_irb14000_msgs)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-DROS=1)           # build using ROS libraries
add_definitions(-DLINUXSOCKETS=1)  # build using LINUX SOCKETS libraries

find_package(Boost REQUIRED COMPONENTS system thread QUIET)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)


add_service_files(
  FILES
  YumiGrasp.srv
)
 
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation std_msgs
)
