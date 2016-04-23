cmake_minimum_required(VERSION 2.8.3)
project(maelstrom_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  DIRECTORY msg
  FILES
  State.msg
  ThrusterForces.msg
  SensorRaw.msg
  PwmRequests.msg
  Joystick.msg
  LightInput.msg
  ArmInput.msg
  JoystickMotionCommand.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs
)
