cmake_minimum_required(VERSION 2.8.3)
project(vortex_msgs)

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
  ThrusterForces.msg
  LightInput.msg
  JoystickMotionCommand.msg
  JoystickArmCommand.msg
  ArmState.msg
  ArmDutyCycle.msg
  EulerOrientations.msg
  Debug.msg
  ThrusterPwm.msg
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
