#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from joystick.msg import Joystick
from joystick.msg import pwm_requests
from joystick.msg import directional_input

class HmiNode:

    def __init__(self):
        rospy.init_node("hmi_node", anonymous=True)

        self.pwm_signals = pwm_requests()
        self.directional_input = directional_input()

        self.pwm_publisher = rospy.Publisher('pwm_requests', pwm_requests, queue_size=10)
        self.uranus_publisher = rospy.Publisher('joy_input', directional_input, queue_size=10)

        rospy.Subscriber('Joystick', Joystick, self.joystick_callback)
        self.rate = rospy.Rate(10)


    def start_node(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def joystick_callback(self, joystick):
        
        self.pwm_signals.pwm1 = joystick.strafe_X
        self.pwm_signals.pwm2 = joystick.strafe_Y
        self.pwm_signals.pwm3 = joystick.turn_X
        self.pwm_signals.pwm4 = joystick.turn_Y
        self.pwm_signals.pwm5 = joystick.ascend
        self.pwm_signals.pwm6 = joystick.descend

        self.directional_input.strafe_X = joystick.strafe_Y
        self.directional_input.strafe_Y = joystick.strafe_X
        self.directional_input.turn_X = joystick.turn_X
        self.directional_input.turn_Y = joystick.turn_Y
        self.directional_input.ascend = joystick.ascend

        self.pwm_publisher.publish(self.pwm_signals)
        self.uranus_publisher.publish(self.directional_input)

if __name__ == '__main__': 
    try:
        print("ready to fuck up")
        hmi_node = HmiNode()
        hmi_node.start_node()
    except rospy.ROSInterruptException:
        pass


  
