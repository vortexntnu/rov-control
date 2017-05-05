cmake_minimum_required(VERSION 2.8.3)
project(camera_selection)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  CATKIN_DEPENDS
  rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)