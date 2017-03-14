#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Quaternion
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
            'sensors/imu/diagnostics',
            String,
            queue_size=10)
        self.pub_euler = rospy.Publisher(
            'sensors/imu/euler',
            Vector3,
            queue_size=10)

        self.bno = BNO055.BNO055(rst='P9_12')
        if not self.bno.begin():
            rospy.logfatal('%s: Failed to initialise BNO055! Is the sensor connected?', rospy.get_name())
            raise rospy.ROSInitException('Failed to initialise BNO055! Is the sensor connected?')

        self.status, self.self_test, self.error = self.bno.get_system_status()
        rospy.loginfo("%s: System status: %s", rospy.get_name(), self.status)
        rospy.loginfo("%s: Self test result (0x0F is normal): 0x%02X", rospy.get_name(),  self.self_test)
        if self.status == 0x01:
            rospy.logwarn("%s: System status: 0x%02X\nSee datasheet section 4.3.59 for the meaning.", rospy.get_name(),  self.status)

        self.sw_v, \
        self.bootloader_v, \
        self.accelerometer_id, \
        self.magnetometer_id, \
        self.gyro_id = self.bno.get_revision()
        rospy.loginfo(( "%s:\n"
        "Software version: %s\n"
        "Bootloader version: %s\n"
        "Accelerometer ID: 0x%02X\n"
        "Magnetometer ID: 0x%02X\n"
        "Gyroscope ID: 0x%02X\n"
        ), rospy.get_name(), self.sw_v, self.bootloader_v, self.accelerometer_id, self.accelerometer_id, self.gyro_id)
        self.talker()

    def talker(self):
        imu_msg = Imu()
        imu_euler_msg = Vector3()
        imu_temp_msg = Temperature()
        imu_mag_msg = MagneticField()
        imu_diag_msg = DiagnosticStatus()

        while not rospy.is_shutdown():
            x, y, z, w = self.bno.read_quaternion()
            heading, roll, pitch = self.bno.read_euler()
            mag_x, mag_y, mag_z = self.bno.read_magnetometer()
            temp = self.bno.read_temp()
            # diag message reading goes here

            imu_msg.orientation = Quaternion(x, y, z, w)
            imu_euler_msg = Vector3(heading, roll, pitch)
            imu_temp_msg.temperature = temp
            imu_mag_msg.magnetic_field = Vector3(mag_x, mag_y, mag_z)
            # Diag message creating goes here

            self.pub_imu.publish(imu_msg)
            self.pub_euler.publish(imu_euler_msg)
            self.pub_imu_temp.publish(imu_temp_msg)
            self.pub_mag.publish(imu_mag_msg)
            # Diag message publish goes here
            rospy.Rate(10).sleep()

if __name__ == '__main__':
    try:
        imu_node = Bno055InterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
