cmake_minimum_required(VERSION 2.8.3)
project(sensor_interface)

find_package(catkin REQUIRED COMPONENTS
    rosserial_arduino
    rosserial_client
    std_msgs
    geometry_msgs
    vortex_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

rosserial_generate_ros_lib(
    PACKAGE rosserial_arduino
    SCRIPT make_libraries.py
)

# add_executable(sensor_interface firmware/sensor_interface.cpp)

# add_dependencies(sensor_interface ${catkin_EXPORTED_TARGETS})
# add_dependencies(sensor_interface ${PROJECT_NAME}_EXPORTED_TARGETS)
# add_dependencies(sensor_interface ${PROJECT_NAME}_generate_messages_cpp)

rosserial_configure_client(
    DIRECTORY firmware
    TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware sensor_interface ALL)
rosserial_add_client_target(firmware sensor_interface-upload)
