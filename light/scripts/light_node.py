#!/usr/bin/env python
import rospy
import Adafruit_BBIO.GPIO as GPIO

class LightNode(object):
    def __init__(self):
        rospy.init_node('light_node')

    def init_publisher():
        pass

    def init_subscriber():
        pass


if __name__ == '__main__':
    try:
        light_node = LightNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass