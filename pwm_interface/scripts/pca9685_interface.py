#!/usr/bin/env python

import Adafruit_PCA9685
import rospy

FREQUENCY = 249


class Pca9685InterfaceNode(object):
    def __init__(self):
        rospy.init_node('pwm_node')
        self.pca9685 = Adafruit_PCA9685.PCA9685()
        self.pca9685.set_pwm_frequency(FREQUENCY)
        self.pca9685.set_all_pwm(0, 0)


if __name__ == '__main__':
    try:
        pwm_node = Pca9685InterfaceNode()
        rospy.spin()
    except:
        pass
