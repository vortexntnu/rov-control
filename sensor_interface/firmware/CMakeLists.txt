cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

generate_arduino_firmware(sensor_interface
    SRCS sensor_interface.cpp I2Cdev.cpp MS5837.cpp Adafruit_BNO055.cpp ${ROS_LIB_DIR}/time.cpp
#     HDRS Adafruit_BNO055.h
    BOARD mega2560
    PORT /dev/ttyACM0
)
