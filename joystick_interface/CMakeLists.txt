cmake_minimum_required(VERSION 2.8.3)
project(joystick_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  maelstrom_msgs
)

add_message_files(
  FILES
  Joystick.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS rospy message_runtime maelstrom_msgs)
