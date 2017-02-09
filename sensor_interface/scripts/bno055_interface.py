#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

from Adafruit_BNO055 import BNO055

class Bno055InterfaceNode(object):
    def __init__(self):
        rospy.init_node('imu_node')
        self.bno = BNO055.BNO055(rst='P9_12')

        self.pub_imu = rospy.Publisher('imu/data', Imu, queue_size=10)
        self.pub_mag = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
        self.pub_imu_temp = rospy.Publisher('imu/temperature', Temperature, queue_size=10)
        self.pub_calibration = rospy.Publisher('imu/calibration', String, queue_size=10)
        self.pub_euler = rospy.Publisher('imu/euler', Vector3, queue_size=10)

if __name__ == '__main__':
    try:
        imu_node = Bno055InterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
