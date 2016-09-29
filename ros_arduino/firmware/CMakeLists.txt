cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
#add_definitions(-DUSB_CON)




generate_arduino_firmware(ros_arduino
    SRCS ros_arduino.cpp I2Cdev/I2Cdev.cpp MS5837.cpp Adafruit_BNO055.h Adafruit_BNO055.cpp ${ROS_LIB_DIR}/time.cpp
    BOARD mega2560
    PORT /dev/ttyACM0

#    SRCS hello.cpp ${ROS_LIB_DIR}/time.cpp
#       SRCS hello.cpp ${ROS_LIB_DIR}/time.cpp
#	ros_arduino.cpp
#	I2Cdev/I2Cdev.cpp
#	MS5837.cpp
)
