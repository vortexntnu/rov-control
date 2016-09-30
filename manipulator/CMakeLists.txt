cmake_minimum_required(VERSION 2.8.3)
project(manipulator)

#find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  vortex_msgs
)

add_message_files(
   FILES
   ArmState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  message_runtime
  vortex_msgs
)
