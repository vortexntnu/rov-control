# rov-control [![Build Status](https://travis-ci.com/vortexntnu/rov-control.svg?branch=master)](https://travis-ci.com/vortexntnu/rov-control)
The control system for Vortex NTNU's ROVs. The repository contains several ROS packages.

## Building
The easiest way to quickly get started building the project is to use the accompanying Docker container.
By running `docker run -it -v $(pwd):/catkin_ws/src/ vortexntnu/rov-control:latest /bin/bash`, when the current
directory is set to the root of this package, you will get a container all set up with ROS and the required
dependencies. You can then run the following commands to build and test the package:

``` shell
source /opt/ros/kinetic/setup.bash
source /catkin_ws/devel/setup.bash
cd /catkin_ws

# Build the packages
catkin_make

# Build the tests
catkin_make tests

# Run the tests
catkin_make run_tests

# Run roslint
catkin_make roslint
```

## Launching
* `roslaunch vortex maelstrom.launch` to start the system for the Maelstrom ROV.
* `roslaunch vortex terrapin.launch` to start the system for the Terrapin ROV.
* `rosrun rqt_reconfigure rqt_reconfigure` to open the dynamic reconfigure GUI window.

## Testing/linting
Tests run automatically on a Travis server. Run tests manually with `catkin_make run_tests`,
and check test results with `catkin_test_results`. Linting is done automatically when running tests,
but you can also lint manually with `catkin_make roslint`.

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

### roslint
roslint is used for linting of the ROS C++/Python code.

`sudo apt install ros-kinetic-roslint`

## Preferred workflow
* Create a feature branch out of `master` for each new feature, solved issue, significant refactor, etc.
* Open a pull request to have your branch merged back into `master`.
