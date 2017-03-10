#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from Adafruit_BNO055 import BNO055

class Bno055InterfaceNode(object):
    def __init__(self):
        rospy.init_node('imu_node')

        self.pub_imu = rospy.Publisher(
            'sensors/imu/data',
            Imu,
            queue_size=10)
        self.pub_mag = rospy.Publisher(
            'sensors/imu/mag',
            MagneticField,
            queue_size=10)
        self.pub_imu_temp = rospy.Publisher(
            'sensors/imu/temperature',
            Temperature,
            queue_size=10)
        self.pub_diagnostics = rospy.Publisher(
            'imu/calibration',
            String,
            queue_size=10)
        self.pub_euler = rospy.Publisher(
            'imu/euler',
            Vector3,
            queue_size=10)

        self.bno = BNO055.BNO055(rst='P9_12')
        if not self.bno.begin():
            rospy.logfatal('Failed to initialise BNO055! Is the sensor connected?')
            raise rospy.ROSInitException('Failed to initialise BNO055! Is the sensor connected?')

        self.status, self.self_test, self.error = self.bno.get_system_status()
        rospy.loginfo("System status: %s", self.status)
        rospy.loginfo("Self test result (0x0F is normal): 0x%02X", self.self_test)
        if self.status == 0x01:
            rospy.logwarn("System status: 0x%02X\nSee datasheet section 4.3.59 for the meaning.", self.status)

        self.sw_v, \
        self.bootloader_v, \
        self.accelerometer_id, \
        self.magnetometer_id, \
        self.gyro_id = self.bno.get_revision()

if __name__ == '__main__':
    try:
        imu_node = Bno055InterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
