cmake_minimum_required(VERSION 2.8.3)
project(hw_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  vortex_msgs
  cmake_modules
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  message_runtime
  vortex_msgs
  roscpp
  sensor_msgs
)
