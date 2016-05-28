cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
#add_definitions(-DUSB_CON)




generate_arduino_firmware(ros_arduino
  SRCS
	ros_arduino.cpp
	${ROS_LIB_DIR}/time.cpp
	I2Cdev/I2Cdev.cpp
	MS5837.cpp
  
  #BOARD mega2560
  #BOARD uno
  BOARD nano328
  
  PORT /dev/ttyUSB0
  #PORT /dev/ttyACM0
  #PORT /dev/ttyS0
)
