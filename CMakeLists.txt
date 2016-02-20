cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware ros_arduino ALL)
rosserial_add_client_target(firmware ros_arduino-upload)


findPackage(message_generation)
catkin_package(CATKIN_DEPENDS message_runtime)
add_message_files(FILES pwm_requests.msg)


