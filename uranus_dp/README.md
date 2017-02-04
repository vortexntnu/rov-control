# Uranus DP
A ROV control system written in ROS.

## Launching
* `roslaunch uranus_dp maelstrom.launch` to start the system for the Maelstrom ROV.
* `rosrun rqt_reconfigure rqt_reconfigure` to open the dynamic reconfigure GUI window.

## Dependencies
* The [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) C++ library
`sudo apt install libeigen-dev`
* The [eigen_conversions](http://wiki.ros.org/eigen_conversions) ROS package
`sudo apt install ros-kinetic-eigen-conversions`

## Control modes
Several control modes are implemented:
* `OPEN_LOOP`: Joystick commands are directly mapped to forces and torques on the ROV.
* `SIXDOF`: Stationkeeping (setpoint regulation) of six degrees of freedom. Joystick inputs move setpoints.
* `RPY_DEPTH`: Setpoint regulation of roll, pitch, yaw, and depth. Corresponding joystick inputs move setpoints. Joystick inputs for forward/sideways motion are executed as in open loop.
* `DEPTH_HOLD`: Setpoint regulation of depth. All other degrees of freedom are run in open loop.
