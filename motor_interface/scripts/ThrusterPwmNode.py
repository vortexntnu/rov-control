#!/usr/bin/env python
# -*- coding: utf-8 -*-

import LookupTable
import Adafruit_PCA9685
import rospy

from vortex_msgs.msg import ThrusterForces
from vortex_msgs.msg import ThrusterPwm

def callback(ForceInput):
    # Calculate PWM signals corresponding to each commanded thruster force
    PwmStatusMsg.pwm1 = int(LookupTable.ForceToPwm(ForceInput.F1))
    PwmStatusMsg.pwm2 = int(LookupTable.ForceToPwm(ForceInput.F2))
    PwmStatusMsg.pwm3 = int(LookupTable.ForceToPwm(ForceInput.F3))
    PwmStatusMsg.pwm4 = int(LookupTable.ForceToPwm(ForceInput.F4))
    PwmStatusMsg.pwm5 = int(LookupTable.ForceToPwm(ForceInput.F5))
    PwmStatusMsg.pwm6 = int(LookupTable.ForceToPwm(ForceInput.F6))

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
    pwm_zero_newton = int(LookupTable.ForceToPwm(0))
    pwm.set_pwm(0, 0, pwm_zero_newton)
    pwm.set_pwm(1, 0, pwm_zero_newton)
    pwm.set_pwm(2, 0, pwm_zero_newton)
    pwm.set_pwm(3, 0, pwm_zero_newton)
    pwm.set_pwm(4, 0, pwm_zero_newton)
    pwm.set_pwm(5, 0, pwm_zero_newton)

if __name__ == '__main__':
    rospy.init_node('PwmNode', anonymous=False)
    init_pwm()

    PwmStatusPub = rospy.Publisher('pwm_status', ThrusterPwm, queue_size=10)
    rospy.Subscriber("thruster_forces", ThrusterForces, callback)

    print "motor_interface: Launching node PwmNode"
    rospy.spin()
