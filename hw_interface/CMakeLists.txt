cmake_minimum_required(VERSION 2.8.3)
project(hw_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  vortex_msgs
  cmake_modules
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  message_runtime
  vortex_msgs
  roscpp
  sensor_msgs
)

include_directories(./src/IMUNode/I2Cdev/ ./src/IMUNode/MPU6050 ./src/IMUNode/MS5803 )

find_library( bcm2835 bcm2835 )


add_executable(IMUNode src/IMUNode/IMUNode.cpp
                       src/IMUNode/I2Cdev/I2Cdev.cpp
                       src/IMUNode/MPU6050/MPU6050.cpp
                       src/IMUNode/MS5803/MS5803.cpp
)

target_link_libraries(IMUNode ${catkin_LIBRARIES} bcm2835 )


