cmake_minimum_required(VERSION 2.8.3)
project(motor_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  vortex_msgs
  std_msgs
  message_generation
)

add_service_files(FILES
    ThrusterDisable.srv)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  rospy
  vortex_msgs
  std_msgs message_runtime
)
