cmake_minimum_required(VERSION 2.8.3)
project(joystick)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES
   Joystick.msg
   PwmRequests.msg
   DirectionalInput.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs message_runtime)
