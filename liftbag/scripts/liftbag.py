#!/usr/bin/env python
import rospy
import socket
import sys

from std_msgs.msg import String


class LiftbagNode(object):
    def __init__(self):
        rospy.init_node('liftbag_node')
        self.sub = rospy.Subscriber('liftbag_release', String, self.callback)

    def callback(self, msg):
        # Creating a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connecting the socket to the port where ESP8266 is listening
        server_address = ('192.168.4.1', 81)
        print >>sys.stderr, 'connectin to %s port %s' % server_address
        sock.connect(server_address)

        print >>sys.stderr, 'closing socket'
        sock.close()


if __name__ == '__main__':
    try:
        liftbag_node = LiftbagNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
