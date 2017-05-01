#!/usr/bin/env python

import rospy


class Pca9685InterfaceNode(object):
    def __init__(self):
        rospy.init_node('pwm_node')


if __name__ == '__main__':
    try:
        pwm_node = Pca9685InterfaceNode()
        rospy.spin()
    except:
        pass
