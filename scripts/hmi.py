#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from joystick.msg import joystick
from joystick.msg import pwm_requests

    


class HmiNode:

    def __init__(self):
        self.pwm_signals = pwm_requests()
        self.pwm_publisher = rospy.Publisher('pwm_requests', pwm_requests, queue_size=10)
        rospy.Subscriber('control', control, self.controller_callback)

        rospy.init_node("hmi_node", anonymous=True)
        self.rate = rospy.Rate(10)


    def start_node(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def controller_callback(self, control):
        self.pwm_signals.pwm1 = control.strafe_X
        self.pwm_signals.pwm2 = control.strafe_Y
        self.pwm_signals.pwm3 = control.turn_X
        self.pwm_signals.pwm4 = control.turn_Y
        self.pwm_signals.pwm5 = control.ascend
        self.pwm_signals.pwm6 = control.descend

        self.pwm_publisher.publish(self.pwm_signals)

if __name__ == '__main__': 
    try:
        hmi_node = HmiNode()
        hmi_node.start_node()
    except rospy.ROSInterruptException:
        pass


  
