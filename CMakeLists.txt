cmake_minimum_required(VERSION 2.8.3)
project(joystick)

# find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  maelstrom_msgs
)

add_message_files(
  FILES
  Joystick.msg
)

catkin_package(CATKIN_DEPENDS rospy message_runtime maelstrom_msgs)
