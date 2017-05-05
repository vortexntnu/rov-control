#!/usr/bin/env python
import rospy
import Adafruit_BBIO.GPIO as GPIO

GPIO_PIN_MAP = rospy.get_param('/camera/gpio_pins')

class CameraSelection(object):
    def __init__(self):
        rospy.init_node('camera_selection')

        #Set pin as output
        for key in GPIO_PIN_MAP:
            GPIO.setup(GPIO_PIN_MAP[key], GPIO.OUT)

        #Set to ground
        for key in GPIO_PIN_MAP:
        	GPIO.output(GPIO_PIN_MAP[light], GPIO.LOW)


if __name__ == '__main__':
    try:
        camera_selection = CameraSelection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

