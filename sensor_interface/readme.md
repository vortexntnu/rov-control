This repository holds the code which will run a ROS node on the arduino.

In order to act as a ROS node a translator node must be running on the host device.
The translator node is found in the rosserial package which can be installed with

 $sudo apt-get install ros-{ROS_VERSION}-rosserial

To run the translator node it must also be supplied with an argument which tells it where to look for the arduino.
For an arduino located at /dev/ttyACM0 we will run the following to bring up the
translator node:

 $rosrun rosserial_python serial_node.py /dev/ttyACM0

Build instructions:
In order to compile this sketch you must have the arduino IDE or similar, and rosserial-arduino which you can get with

 $sudo apt-get install ros-{ROS_VERSION}-rosserial-arduino

The package should build when your run 'catkin_make'
If you want to build only this package run 'catkin_make sensor_interface_firmware_sensor_interface'

In order to upload this node to the Arduino run 'catkin_make sensor_interface_firmware_sensor_interface-upload'
Make sure you specify your arduino board and serial port in 'sensor_interface/firmware/CMakeLists.txt'


relevant tutorials:

http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

http://wiki.ros.org/rosserial_arduino/Tutorials/CMake
