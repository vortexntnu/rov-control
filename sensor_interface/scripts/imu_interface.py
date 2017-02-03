#!/usr/bin/env python

import rospy

from Adafruit_BNO055 import BNO055

class ImuInterfaceNode(object):
    def __init__(self):
        rospy.init_node('imu_node')
        self.bno = BNO055.BNO055(rst='P9_12')
        


if __name__ == '__main__':
    try:
        joystick_node = JoystickInterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
