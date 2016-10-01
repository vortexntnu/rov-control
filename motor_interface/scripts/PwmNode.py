#!/usr/bin/env python
# -*- coding: utf-8 -*-

import ForceToPwmLookup
import Adafruit_PCA9685
import rospy

from vortex_msgs.msg import ThrusterForces
from vortex_msgs.msg import ThrusterPwm

def callback_thruster(ForceInput):
    # Publish status message
    PwmStatusMsg.pwm1 = int(ForceToPwmLookup.ForceToPwm(ForceInput.F1))
    PwmStatusMsg.pwm2 = int(ForceToPwmLookup.ForceToPwm(ForceInput.F2))
    PwmStatusMsg.pwm3 = int(ForceToPwmLookup.ForceToPwm(ForceInput.F3))
    PwmStatusMsg.pwm4 = int(ForceToPwmLookup.ForceToPwm(ForceInput.F4))
    PwmStatusMsg.pwm5 = int(ForceToPwmLookup.ForceToPwm(ForceInput.F5))
    PwmStatusMsg.pwm6 = int(ForceToPwmLookup.ForceToPwm(ForceInput.F6))
    PwmStatusPub.publish(PwmStatusMsg)

    # Set PWM outputs
    pwm.set_pwm(0, 0, PwmStatusMsg.pwm1)
    pwm.set_pwm(1, 0, PwmStatusMsg.pwm2)
    pwm.set_pwm(2, 0, PwmStatusMsg.pwm3)
    pwm.set_pwm(3, 0, PwmStatusMsg.pwm4)
    pwm.set_pwm(4, 0, PwmStatusMsg.pwm5)
    pwm.set_pwm(5, 0, PwmStatusMsg.pwm6)

def init_pwm():
    # Initialise the PCA9685 using the default address (0x40).
    pwm = Adafruit_PCA9685.PCA9685()

    # 260 Hz gives a pulse length of roughly 3800 microseconds
    # Our motor controllers require min. 2000 microseconds pulse length
    pwm.set_pwm_freq(260)

    # Til å sende debugmeldinger
    PwmStatusMsg = ThrusterPwm()

    # Initialize all thrusters to 0 newton
    pwm_zero_newton = int(ForceToPwmLookup.ForceToPwm(0))
    pwm.set_pwm(0, 0, pwm_zero_newton)
    pwm.set_pwm(1, 0, pwm_zero_newton)
    pwm.set_pwm(2, 0, pwm_zero_newton)
    pwm.set_pwm(3, 0, pwm_zero_newton)
    pwm.set_pwm(4, 0, pwm_zero_newton)
    pwm.set_pwm(5, 0, pwm_zero_newton)

if __name__ == '__main__':
    rospy.init_node('PwmNode', anonymous=True)
    init_pwm()

    PwmStatusPub = rospy.Publisher('pwm_status', ThrusterPwm, queue_size=10)
    rospy.Subscriber("thruster_forces", ThrusterForces, callback_thruster)

    print "motor_interface: Launching node PwmNode"
    rospy.spin()
