#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from joystick.msg import joystick
from joystick.msg import pwm_requests
from uranus_dp.msg import JoystickUranus #hehehe

class HmiNode:

    def __init__(self):
        rospy.init_node("hmi_node", anonymous=True)

        self.pwm_signals = pwm_requests()
        self.uranus_signals = JoystickUranus()

        self.pwm_publisher = rospy.Publisher('pwm_requests', pwm_requests, queue_size=10)
        self.uranus_publisher = rospy.Publisher('joy_input', JoystickUranus, queue_size=10)

        rospy.Subscriber('joystick', joystick, self.joystick_callback)
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

        self.uranus_signals.surge = joystick.strafe_Y
        self.uranus_signals.sway = joystick.strafe_X
        self.uranus_signals.yaw = joystick.turn_X
        self.uranus_signals.heave = joystick.ascend
        self.uranus_signals.roll = 0
        self.uranus_signals.pitch = joystick.turn_Y

        self.pwm_publisher.publish(self.pwm_signals)
        self.uranus_publisher.publish(self.uranus_signals)

if __name__ == '__main__': 
    try:
        hmi_node = HmiNode()
        hmi_node.start_node()
    except rospy.ROSInterruptException:
        pass


  
