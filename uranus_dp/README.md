# Uranus DP
A ROV control system written in ROS.

## Usage
* `roslaunch uranus_dp maelstrom.launch` to start the system for the Maelstrom ROV.
* `rosrun rqt_reconfigure rqt_reconfigure` to open the dynamic reconfigure GUI window.

## Dependencies
* The [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) C++ library
`sudo apt install libeigen-dev`
* The [eigen_conversions](http://wiki.ros.org/eigen_conversions) ROS package
`sudo apt install ros-kinetic-eigen-conversions`
