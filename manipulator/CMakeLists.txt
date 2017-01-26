cmake_minimum_required(VERSION 2.8.3)
project(manipulator)

#find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  vortex_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  message_runtime
  vortex_msgs
)
