#!/usr/bin/env python
import rospy
import serial
import Adafruit_BBIO.UART as UART
from vortex_msgs.msg import ContainerID

SPIN_RATE = 10


class Hc05InterfaceNode(object):
    def __init__(self):
        rospy.init_node('bluetooth_node')
        self.port = "/dev/ttyO4"
        self.init_serial()
        self.init_publisher()
        rospy.loginfo('Initialized Bluetooth.')
        self.read_bluetooth_data()

    def init_serial(self):
        UART.setup("UART4")
        self.ser = serial.Serial(port=self.port,
                                 baudrate=9600,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=1)
        if not self.ser.is_open:
            rospy.logwarn("Unable to open %s", self.ser)

    def init_publisher(self):
        self.pub_bluetooth = rospy.Publisher(
            'bluetooth/container_id',
            ContainerID,
            queue_size=10)

    def read_bluetooth_data(self):
        while not rospy.is_shutdown():
            BT_msg = self._readline()
            if len(BT_msg) > 0:
                self.pub_bluetooth.publish(BT_msg)
            rospy.Rate(SPIN_RATE).sleep()

    def _readline(self):
        eol = b'\r'
        line = bytearray()
        while True:
            c = self.ser.read(1)
            if c:
                line += c
                if line[-len(eol):] == eol:
                    break
            else:
                break
        return bytes(line)


if __name__ == '__main__':
    try:
        bluetooth_node = Hc05InterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except:
        rospy.logerr('Unexpected error, shutting down.')
