#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from controller.msg import control


mux_pub = rospy.Publisher('pwm_signal_input', Int32, queue_size=10) 

def controller_callback(data):
    mux_pub.publish(data.strafe_X)
    print(data.strafe_X)


if __name__ == '__main__': 
    rospy.init_node('mux_node', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber('control', control, controller_callback)
    rospy.spin()
    
