#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import Adafruit_PCA9685
from numpy import interp

import lookup_table
from vortex_msgs.msg import ThrusterForces, ThrusterPwm

BITS_PER_PERIOD               = 4096.0 # 12 bit PWM
FREQUENCY                     = 260    # Max 500 Hz (min total pulse width 2000 microseconds)
PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0/FREQUENCY
NEUTRAL_PULSE_WIDTH           = interp(0, lookup_table.thruster_force, lookup_table.pulse_width)

def pulse_width_in_bits(force):
    pulse_width_in_microseconds = interp(force, lookup_table.thruster_force, lookup_table.pulse_width)
    return int(round(pulse_width_in_microseconds))
    normalized_duty_cycle = pulse_width_in_microseconds/PERIOD_LENGTH_IN_MICROSECONDS
    return int(round(BITS_PER_PERIOD * normalized_duty_cycle))

def callback(desired_forces):
    pwm_state = ThrusterPwm()

    # Calculate PWM signals corresponding to each commanded thruster force
    pwm_state.pwm1 = pulse_width_in_bits(desired_forces.F1)
    pwm_state.pwm2 = pulse_width_in_bits(desired_forces.F2)
    pwm_state.pwm3 = pulse_width_in_bits(desired_forces.F3)
    pwm_state.pwm4 = pulse_width_in_bits(desired_forces.F4)
    pwm_state.pwm5 = pulse_width_in_bits(desired_forces.F5)
    pwm_state.pwm6 = pulse_width_in_bits(desired_forces.F6)

    # Set PWM outputs
    # pca9685.set_pwm(0, 0, pwm_state.pwm1)
    # pca9685.set_pwm(1, 0, pwm_state.pwm2)
    # pca9685.set_pwm(2, 0, pwm_state.pwm3)
    # pca9685.set_pwm(3, 0, pwm_state.pwm4)
    # pca9685.set_pwm(4, 0, pwm_state.pwm5)
    # pca9685.set_pwm(5, 0, pwm_state.pwm6)

    # Publish outputs for debug
    pub.publish(pwm_state)

def init():
    pass
    # Initialise the PCA9685 using the default address (0x40)
    # pca9685 = Adafruit_PCA9685.PCA9685()

    # 260 Hz gives a pulse length of roughly 3800 microseconds
    # Our motor controllers require min. 2000 microseconds pulse length
    # Max frequency is 500 Hz
    # pca9685.set_pwm_freq(FREQUENCY)

    # Set all thrusters to zero
    # pca9685.set_pwm(0, 0, NEUTRAL_PULSE_WIDTH)
    # pca9685.set_pwm(1, 0, NEUTRAL_PULSE_WIDTH)
    # pca9685.set_pwm(2, 0, NEUTRAL_PULSE_WIDTH)
    # pca9685.set_pwm(3, 0, NEUTRAL_PULSE_WIDTH)
    # pca9685.set_pwm(4, 0, NEUTRAL_PULSE_WIDTH)
    # pca9685.set_pwm(5, 0, NEUTRAL_PULSE_WIDTH)

    # Create a message object


if __name__ == '__main__':
    rospy.init_node('ThrusterPwmNode', anonymous=False)
    init()

    pub = rospy.Publisher('pwm_state', ThrusterPwm, queue_size=10)
    rospy.Subscriber('thruster_forces', ThrusterForces, callback)

    print "motor_interface: Launching node ThrusterPwmNode"

    rospy.spin()
