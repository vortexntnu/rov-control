# sensor_interface
This package is the interface between the ROV control system running on a Raspberry Pi and the sensors attached to the ROV. The sensors are read from an Arduino board, which acts as a ROS node by means of `rosserial_arduino`.

## Running the code
* Build and upload the Arduino code
* Run a translator node on the host (Raspberry Pi):
`rosrun rosserial_python serial_node.py /dev/ttyUSB0`
(Note: May also be `/dev/ttyACM0` or something else.)

## Building and uploading the code
* Build the package
`catkin_make` or `catkin_make sensor_interface_firmware_sensor_interface`
* Upload to Arduino
`catkin_make sensor_interface_firmware_sensor_interface-upload`
Make sure that the correct Arduino board and serial port is specified in `firmware/CMakeLists.txt`.

## Dependencies
* [rosserial](http://wiki.ros.org/rosserial)
`sudo apt-get install ros-kinetic-rosserial`
* [rosserial_arduino](http://wiki.ros.org/rosserial_arduino)
`sudo apt-get install ros-kinetic-rosserial-arduino`

<!-- ## Notes -->
