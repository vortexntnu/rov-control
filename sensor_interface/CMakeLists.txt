cmake_minimum_required(VERSION 2.8.3)
project(sensor_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  vortex_msgs
)

catkin_package(

  )

include_directories(
  ${catkin_INCLUDE_DIRS}

  )
