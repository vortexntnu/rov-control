#!/usr/bin/env python

import rospy
import Adafruit_PCA9685
from numpy import interp

from vortex_msgs.msg import ThrusterForces, ThrusterPwm

BITS_PER_PERIOD               = 4096.0 # 12 bit PWM
FREQUENCY                     = 249    # Max 400 Hz
FREQUENCY_MEASURED            = 251.2  # Use this for better precision
PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0/FREQUENCY_MEASURED

T100_thrust      = rospy.get_param('/thrust')
T100_pulse_width = rospy.get_param('/pulse_width')
num_thrusters    = rospy.get_param('/num_thrusters')

# Initialise the PCA9685 using the default address (0x40)
pca9685 = Adafruit_PCA9685.PCA9685()

def pulse_width_in_bits(force):
    pulse_width_in_microseconds = interp(force, T100_thrust, T100_pulse_width)
    normalized_duty_cycle = pulse_width_in_microseconds/PERIOD_LENGTH_IN_MICROSECONDS
    return int(round(BITS_PER_PERIOD * normalized_duty_cycle))

def callback(msg):
    pwm_state = [None]*num_thrusters
    for i in range(num_thrusters):
        pwm_state[i] = pulse_width_in_bits(msg.thrust[i])
        pca9685.set_pwm(i, 0, pwm_state[i])

    # Publish outputs for debug
    pwm_msg = ThrusterPwm()
    pwm_msg.pwm = pwm_state
    pub.publish(pwm_state)

def init():
    pca9685.set_pwm_freq(FREQUENCY)

    neutral_pulse_width = pulse_width_in_bits(0)
    for i in range(num_thrusters):
        pca9685.set_pwm(i, 0, neutral_pulse_width)

if __name__ == '__main__':
    rospy.init_node('motor_interface', anonymous=False)
    init()

    pub = rospy.Publisher('pwm_state', ThrusterPwm, queue_size=10)
    rospy.Subscriber('thruster_forces', ThrusterForces, callback)

    print 'Launching node motor_interface'
    rospy.spin()
