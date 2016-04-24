#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from maelstrom_msgs.msg import Joystick
from maelstrom_msgs.msg import PwmRequests
from maelstrom_msgs.msg import DirectionalInput

class HmiNode:

    def __init__(self):
        rospy.init_node("hmi_node", anonymous=True)

        self.pwm_signals = PwmRequests()
        self.directional_input = DirectionalInput()

        self.pwm_publisher = rospy.Publisher('pwm_requests', PwmRequests, queue_size=10)
        self.uranus_publisher = rospy.Publisher('joy_input', DirectionalInput, queue_size=10)

        rospy.Subscriber('joystick', Joystick, self.joystick_callback)
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
        self.directional_input.ascend = joystick.ascend - joystick.descend

        if(joystick.free_roam): 
            self.directional_input.control_mode = 0

        if(joystick.hold_position): 
            self.directional_input.control_mode = 1

        self.pwm_publisher.publish(self.pwm_signals)
        self.uranus_publisher.publish(self.directional_input)

if __name__ == '__main__': 
    try:
        print("ready to fuck up")
        hmi_node = HmiNode()
        hmi_node.start_node()
    except rospy.ROSInterruptException:
        pass


  
