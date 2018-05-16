cmake_minimum_required(VERSION 2.8.3)
project(liftbag)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

install(PROGRAMS
  scripts/liftbag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
