#!/usr/bin/env python

import rospy
import Adafruit_PCA9685
from numpy import interp
from vortex_msgs.msg import ThrusterForces, ThrusterPwm

class MotorInterface(object):
    def __init__(self):
        self.BITS_PER_PERIOD               = 4096.0 # 12 bit PWM
        self.FREQUENCY                     = 249    # Max 400 Hz
        self.FREQUENCY_MEASURED            = 251.2  # Use this for better precision
        self.PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0/self.FREQUENCY_MEASURED

        self.T100_thrust      = rospy.get_param('/thrusters/characteristics/thrust')
        self.T100_pulse_width = rospy.get_param('/thrusters/characteristics/pulse_width')
        self.num_thrusters    = rospy.get_param('/propulsion/thrusters/num')

        # Initialize the PCA9685 using the default address (0x40)
        # self.pca9685 = Adafruit_PCA9685.PCA9685()
        # pca9685.set_pwm_freq(FREQUENCY)

        # Initialize outputs to zero newton
        neutral_pulse_width = self.pulse_width_in_bits(0)
        # for i in range(num_thrusters):
            # pca9685.set_pwm(i, 0, neutral_pulse_width)

        rospy.init_node('motor_interface', anonymous=False)
        self.pub = rospy.Publisher('pwm_state', ThrusterPwm, queue_size=10)
        self.sub = rospy.Subscriber('thruster_forces', ThrusterForces, self.callback)

        print 'Launching node motor_interface at', self.FREQUENCY, 'Hz'
        rospy.spin()

    def pulse_width_in_bits(self, force):
        pulse_width_in_microseconds = interp(force, self.T100_thrust, self.T100_pulse_width)
        normalized_duty_cycle = pulse_width_in_microseconds/self.PERIOD_LENGTH_IN_MICROSECONDS
        return int(round(self.BITS_PER_PERIOD * normalized_duty_cycle))

    def callback(self, msg):
        pwm_state = [None]*self.num_thrusters
        for i in range(self.num_thrusters):
            pwm_state[i] = self.pulse_width_in_bits(msg.thrust[i])
            # pca9685.set_pwm(i, 0, pwm_state[i])

        # Publish outputs for debug
        pwm_msg = ThrusterPwm()
        pwm_msg.pwm = pwm_state
        self.pub.publish(pwm_state)

if __name__ == '__main__':
    try:
        motor_node = MotorInterface()
    except rospy.ROSInterruptException:
        pass
