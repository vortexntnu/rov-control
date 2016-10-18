#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import Adafruit_PCA9685
from numpy import interp

from vortex_msgs.msg import ThrusterForces, ThrusterPwm

BITS_PER_PERIOD               = 4096.0 # 12 bit PWM
FREQUENCY                     = 249    # Max 400 Hz
PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0/FREQUENCY

# Load thruster characteristics
T100_thrust      = rospy.get_param('/thrust')
T100_pulse_width = rospy.get_param('/pulse_width')

# Initialise the PCA9685 using the default address (0x40)
pca9685 = Adafruit_PCA9685.PCA9685()

# Create empty PWM state message
pwm_state = ThrusterPwm()

def pulse_width_in_bits(force):
    pulse_width_in_microseconds = interp(force, T100_thrust, T100_pulse_width)
    normalized_duty_cycle = pulse_width_in_microseconds/PERIOD_LENGTH_IN_MICROSECONDS
    return int(round(BITS_PER_PERIOD * normalized_duty_cycle))

def callback(desired_thrust):
    # Calculate PWM signals corresponding to each commanded thruster force
    pwm_state.pwm1 = pulse_width_in_bits(desired_thrust.F1)
    pwm_state.pwm2 = pulse_width_in_bits(desired_thrust.F2)
    pwm_state.pwm3 = pulse_width_in_bits(desired_thrust.F3)
    pwm_state.pwm4 = pulse_width_in_bits(desired_thrust.F4)
    pwm_state.pwm5 = pulse_width_in_bits(desired_thrust.F5)
    pwm_state.pwm6 = pulse_width_in_bits(desired_thrust.F6)

    # Set PWM outputs
    pca9685.set_pwm(0, 0, pwm_state.pwm1)
    pca9685.set_pwm(1, 0, pwm_state.pwm2)
    pca9685.set_pwm(2, 0, pwm_state.pwm3)
    pca9685.set_pwm(3, 0, pwm_state.pwm4)
    pca9685.set_pwm(4, 0, pwm_state.pwm5)
    pca9685.set_pwm(5, 0, pwm_state.pwm6)

    # Publish outputs for debug
    pub.publish(pwm_state)

def init():
    pca9685.set_pwm_freq(FREQUENCY)

    neutral_pulse_width = pulse_width_in_bits(0)
    pca9685.set_pwm(0, 0, neutral_pulse_width)
    pca9685.set_pwm(1, 0, neutral_pulse_width)
    pca9685.set_pwm(2, 0, neutral_pulse_width)
    pca9685.set_pwm(3, 0, neutral_pulse_width)
    pca9685.set_pwm(4, 0, neutral_pulse_width)
    pca9685.set_pwm(5, 0, neutral_pulse_width)

if __name__ == '__main__':
    rospy.init_node('motor_interface', anonymous=False)
    init()

    pub = rospy.Publisher('pwm_state', ThrusterPwm, queue_size=10)
    rospy.Subscriber('thruster_forces', ThrusterForces, callback)

    print 'Launching node motor_interface'
    rospy.spin()
