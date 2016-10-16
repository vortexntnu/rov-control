# motor_interface
This package is the interface between the ROV control system running on a Raspberry Pi and the [BlueRobotics ESCs](https://www.bluerobotics.com/store/thrusters/besc-30-r1/). The interfacing is done through the [Adafruit PCA9685](https://www.adafruit.com/product/815) 16-channel 12-bit PWM board.

## Usage
Run this package independently with `rosrun motor_interface motor_interface.py`. In most cases it should be run through a ROS launch script.

## Dependencies
* [Adafruit Python PCA9685](https://github.com/adafruit/Adafruit_Python_PCA9685)
`sudo pip install adafruit-pca9685`
* [The Python NumPy library](http://www.numpy.org/)
`sudo apt-get install python-numpy`
* Working [I2C on the Raspberry Pi](https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=104133)

## Notes
The node will only run if the PCA9685 is properly connected to the Raspberry Pi. Connect VCC to a 3.3 V pin, SCL to SCL, SDA to SDA, and ground to ground.
