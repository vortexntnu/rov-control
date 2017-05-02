cmake_minimum_required(VERSION 2.8.3)
project(bluetooth)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  roslint
)

roslint_python()

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
