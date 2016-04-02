cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino)




find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  message_generation
  std_msgs
  geometry_msgs
)


add_message_files(
    FILES
    SensorRaw.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

add_dependencies(ros_arduino_ros_lib
	uranus_dp_generate_messages
	ros_arduino_generate_messages
	joystick_generate_messages
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware ros_arduino ALL)
rosserial_add_client_target(firmware ros_arduino-upload)


