cmake_minimum_required(VERSION 2.8.3)
project(maelstrom_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  State.msg
  ThrusterForces.msg
  SensorRaw.msg
  DirectionalInput.msg
  PwmRequests.msg
  Joystick.msg
  LightInput.msg
  ArmInput.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  message_generation
  geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#target_link_libraries(maelstrom_msgs_node
#  ${catkin_LIBRARIES}
#)

#############
## Testing ##
#############
