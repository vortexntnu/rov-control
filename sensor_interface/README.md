# Sensor Interface
These nodes interface with the IMU and the pressure sensors.

## Dependencies
* [Adafruit python library](https://github.com/adafruit/Adafruit_Python_BNO055) for BNO055.
* [Adafruit BeagleBone IO library](https://learn.adafruit.com/setting-up-io-python-library-on-beaglebone-black/installation-on-ubuntu) 
`pip install Adafruit_BBIO`
* [Python Serial library](https://pythonhosted.org/pyserial/)
`pip install pyserial`

## Physical connections
The BNO055 library uses by default i2c-1, which has SCL on P9_17 and SDA on P9_18. RST is connected to P9_12.
The Bluetooth interface script uses UART4, which has RX on P9_11 and TX on P9_13.
