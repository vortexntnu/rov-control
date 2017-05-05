#!/usr/bin/env python
import rospy
import Adafruit_BBIO.GPIO as GPIO
from vortex_msgs.msg import CameraFeedSelection

PIN_MAP_FEED0 = rospy.get_param('/camera/pin_map_feed0')
PIN_MAP_FEED1 = rospy.get_param('/camera/pin_map_feed1')
PIN_MAP_FEED2 = rospy.get_param('/camera/pin_map_feed2')
PIN_MAP_LIST = [PIN_MAP_FEED0,PIN_MAP_FEED1,PIN_MAP_FEED2]

class CameraSelection(object):
    def __init__(self):
        rospy.init_node('camera_selection')
        self.cam_select_sub = rospy.Subscriber('camera_feed_selection',CameraFeedSelection, self.callback)

        #Set pin as output
        for pin_list in PIN_MAP_LIST:
            for pin in pin_list:
                GPIO.setup(pin, GPIO.OUT)


    def callback(self, msg):
        feed_pin_map = PIN_MAP_LIST[msg.feed]
        for i, level in enumerate(msg.pin_values):
            if level:
                GPIO.output(feed_pin_map[i], GPIO.HIGH)
            else
                GPIO.output(feed_pin_map[i], GPIO.LOW)



if __name__ == '__main__':
    try:
        camera_selection = CameraSelection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

