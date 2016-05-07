cmake_minimum_required(VERSION 2.8.3)
project(hw_interface)

#find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  maelstrom_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  rospy 
  message_runtime 
  maelstrom_msgs
)
