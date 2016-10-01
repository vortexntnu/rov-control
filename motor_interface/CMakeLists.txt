cmake_minimum_required(VERSION 2.8.3)
project(motor_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  vortex_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  message_runtime
  vortex_msgs
  sensor_msgs
)
