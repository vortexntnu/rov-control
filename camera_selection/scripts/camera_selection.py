#!/usr/bin/env python
import rospy
import Adafruit_BBIO.GPIO as GPIO
from vortex_msgs.msg import CameraFeedSelection

GPIO_PIN_MAP = rospy.get_param('/camera/pin_map')

class CameraSelection(object):
    def __init__(self):
        rospy.init_node('camera_selection')
        self.cam_select_sub = rospy.Subscriber('camera_feed_selection',CameraFeedSelection, self.callback)

        #Set pin as output
        for selection_pin in GPIO_PIN_MAP:
            GPIO.setup(GPIO_PIN_MAP[selection_pin], GPIO.OUT)

        #Set to ground
        for selection_pin in GPIO_PIN_MAP:
        	GPIO.output(GPIO_PIN_MAP[selection_pin], GPIO.LOW)

    def callback(self, msg):
        pass


if __name__ == '__main__':
    try:
        camera_selection = CameraSelection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

