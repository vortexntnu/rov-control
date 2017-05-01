# Bluetooth
This node reads ASCII-strings broadcast by the Bluetooth master

## Dependencies
* [Adafruit BeagleBone IO library](https://learn.adafruit.com/setting-up-io-python-library-on-beaglebone-black/installation-on-ubuntu) 
`pip install Adafruit_BBIO`
* [Python Serial library](https://pythonhosted.org/pyserial/)
`pip install pyserial`


## Physical connections
The Bluetooth interface script uses UART4, which has RX on P9_11 and TX on P9_13.
