#!/usr/bin/env python

import rospy

class Ms5837InterfaceNode(object):
    def __init__(self):
        rospy.init_node('pressure_node')
        #Stuff

if __name__ == '__main__':
    try:
        pressure_node = Ms5837InterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
