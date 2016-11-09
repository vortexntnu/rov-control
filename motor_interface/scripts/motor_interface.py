#!/usr/bin/env python

import rospy
import Adafruit_PCA9685
# from numpy import interp
import numpy as np
from vortex_msgs.msg import ThrusterForces, ThrusterPwm

class MotorInterface(object):
    def __init__(self):
        rospy.init_node('motor_interface', anonymous=False)
        self.pub = rospy.Publisher('pwm_state', ThrusterPwm, queue_size=10)
        self.sub = rospy.Subscriber('thruster_forces', ThrusterForces, self.callback)

        self.BITS_PER_PERIOD               = 4096.0 # 12 bit PWM
        self.FREQUENCY                     = 249    # Max 400 Hz
        self.FREQUENCY_MEASURED            = 251.2  # Use this for better precision
        self.PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0/self.FREQUENCY_MEASURED

        self.T100_thrust      = rospy.get_param('/thrusters/characteristics/thrust')
        self.T100_pulse_width = rospy.get_param('/thrusters/characteristics/pulse_width')
        self.num_thrusters    = rospy.get_param('/propulsion/thrusters/num')
        self.max_rate         = rospy.get_param('/thrusters/rate_of_change/max')

        self.thrust_setpoint  = np.zeros(self.num_thrusters) # The setpoint is the desired value (input)
        self.thrust_reference = np.zeros(self.num_thrusters) # The reference is the output value (rate limited)

        # Initialize the PCA9685 using the default address (0x40)
        # self.pca9685 = Adafruit_PCA9685.PCA9685()
        # pca9685.set_pwm_freq(FREQUENCY)

        # Initialize outputs to zero newton
        neutral_pulse_width = self.pulse_width_in_bits(0)
        # for i in range(num_thrusters):
            # pca9685.set_pwm(i, 0, neutral_pulse_width)

        print 'Launching node motor_interface at', self.FREQUENCY, 'Hz'

    def pulse_width_in_bits(self, force):
        pulse_width_in_microseconds = np.interp(force, self.T100_thrust, self.T100_pulse_width)
        normalized_duty_cycle = pulse_width_in_microseconds/self.PERIOD_LENGTH_IN_MICROSECONDS
        return int(round(self.BITS_PER_PERIOD * normalized_duty_cycle))

    def callback(self, msg):
        thrust_setpoint_list = msg.thrust
        self.thrust_setpoint = thrust_setpoint_list

    def set_pwm(self):
        pwm_state = [None]*self.num_thrusters
        for i in range(self.num_thrusters):
            pwm_state[i] = self.pulse_width_in_bits(self.thrust_setpoint[i])
            # pca9685.set_pwm(i, 0, pwm_state[i])

        # Publish outputs for debug
        pwm_msg = ThrusterPwm()
        pwm_msg.header.stamp = rospy.get_rostime()
        pwm_msg.pwm = pwm_state
        self.pub.publish(pwm_msg)

    def spin(self):
        spin_rate = rospy.get_param('/controller/frequency')
        dt = 1.0/spin_rate
        print dt
        r = rospy.Rate(spin_rate)
        while not rospy.is_shutdown():
            rate_of_change = (self.thrust_setpoint - self.thrust_reference)/dt
            # Calculate rate limited output
            for i in range(self.num_thrusters):
                if rate_of_change[i] > self.max_rate:
                    if i == 0:
                        print 'too fast up'
                    self.thrust_reference[i] += dt * self.max_rate
                elif rate_of_change[i] < -self.max_rate:
                    if i == 0:
                        print 'too fast down'
                    self.thrust_reference[i] += dt * self.max_rate
                else:
                    if i == 0:
                        print 'slow enough'
                    self.thrust_reference[i] = self.thrust_setpoint[i]

            # set and publish output
            self.set_pwm()
            r.sleep()

if __name__ == '__main__':
    try:
        motor_interface = MotorInterface()
        motor_interface.spin()
    except rospy.ROSInterruptException:
        pass
