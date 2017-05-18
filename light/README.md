# light
Node for controlling the ROV lights. Reads messages published by the GUI, and controls lights accordingly

## Dependencies
* [Adafruit BeagleBone IO library](https://learn.adafruit.com/setting-up-io-python-library-on-beaglebone-black/installation-on-ubuntu) 
`pip install Adafruit_BBIO`

## Physical connections
Bluetooth light: P8_07
Raman light: P8_08
Front lights: Adafruit PCA-9685 pin 8
