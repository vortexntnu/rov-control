# rov-control [![Build Status](https://travis-ci.org/vortexntnu/rov-control.svg?branch=master)](https://travis-ci.org/vortexntnu/rov-control)
The control system for Vortex NTNU's ROVs. The repository contains several ROS packages.

## Launching
* `roslaunch vortex maelstrom.launch` to start the system for the Maelstrom ROV.
* `rosrun rqt_reconfigure rqt_reconfigure` to open the dynamic reconfigure GUI window.

## Testing
Run all tests at once with
```
rostest vortex integration_test.test &&
rostest vortex_allocator allocator_test.test &&
rostest vortex_controller controller_test.test &&
rostest vortex_estimator estimator_test.test
```

## Dependencies
### Eigen
The [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) C++ library

`sudo apt install libeigen3-dev`
### Eigen conversions ROS package
The [eigen_conversions](http://wiki.ros.org/eigen_conversions) ROS package

`sudo apt install ros-kinetic-eigen-conversions`
### Gtest
The gtest package in the Ubuntu repository is not precompiled, 
so it needs to be built and put in place.
```
sudo apt install libgtest-dev
cd /usr/src/gtest
sudo cmake .
sudo make
sudo cp libg* /usr/lib/
```

## Control modes
Several control modes are implemented:
* `OPEN_LOOP`: Joystick commands are directly mapped to forces and torques on the ROV.
* `SIXDOF`: Stationkeeping (setpoint regulation) of six degrees of freedom. Joystick inputs move setpoints.
* `RPY_DEPTH`: Setpoint regulation of roll, pitch, yaw, and depth. Corresponding joystick inputs move setpoints. Joystick inputs for forward/sideways motion are executed as in open loop.
* `DEPTH_HOLD`: Setpoint regulation of depth. All other degrees of freedom are run in open loop.

## Preferred workflow
* Create a feature branch for each new feature, solved issue, significant refactoring, etc.
* Run any applicable tests before merging.
* Merge into master when you are confident that your changes work as you intended.
* Use `git merge --no-ff your-branch` to preserve history.
