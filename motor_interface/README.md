# motor_interface
This package defines the interface between the ROV motor controllers and the control system.

## Usage
Run this package independently with `rosrun motor_interface motor_interface.py`. In most cases it should be run through a ROS launch script.

## Dependencies
* [Adafruit Python PCA9685](https://github.com/adafruit/Adafruit_Python_PCA9685)
`sudo pip install adafruit-pca9685`
* [The Python NumPy library](http://www.numpy.org/)
`sudo apt-get install python-numpy`
