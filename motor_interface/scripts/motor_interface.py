#!/usr/bin/env python

import rospy
import numpy
import Adafruit_PCA9685
from vortex_msgs.msg import ThrusterForces, ThrusterPwm

class MotorInterface(object):
    def __init__(self):
        rospy.init_node('motor_interface', anonymous=False)
        self.pub = rospy.Publisher('debug/thruster_pwm', ThrusterPwm, queue_size=10)
        self.sub = rospy.Subscriber('thruster_forces', ThrusterForces, self.callback)

        self.PWM_BITS_PER_PERIOD           = 4096.0 # 12 bit PWM
        self.FREQUENCY                     = 249    # Max 400 Hz
        self.FREQUENCY_MEASURED            = 251.2  # Use this for better precision
        self.PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0/self.FREQUENCY_MEASURED

        self.T100_thrust      = rospy.get_param('/thrusters/characteristics/thrust')
        self.T100_pulse_width = rospy.get_param('/thrusters/characteristics/pulse_width')
        self.num_thrusters    = rospy.get_param('/propulsion/thrusters/num')
        self.max_rate         = rospy.get_param('/thrusters/rate_of_change/max')

        self.prev_time = rospy.get_rostime()
        self.is_initialized = False

        # The setpoint is the desired value (input)
        self.thrust_setpoint  = numpy.zeros(self.num_thrusters)
        # The reference is the output value (rate limited)
        self.thrust_reference = numpy.zeros(self.num_thrusters)

        # Initialize the PCA9685 using the default address (0x40)
        # self.pca9685 = Adafruit_PCA9685.PCA9685()
        # pca9685.set_pwm_freq(FREQUENCY)

        # Initialize outputs to zero newton
        neutral_pulse_width = self.pulse_width_in_bits(0)
        # for i in range(num_thrusters):
            # pca9685.set_pwm(i, 0, neutral_pulse_width)

        print 'Launching node motor_interface at', self.FREQUENCY, 'Hz'

    def callback(self, msg):
        if not self.is_initialized:
            self.prev_time = msg.header.stamp
            self.is_initialized = True
            print 'Initialized motor_interface'
            return

        curr_time = msg.header.stamp
        dt = (curr_time - self.prev_time).to_sec()

        if dt == 0:
            rospy.logwarn('Zero time difference between thruster messages to motor_interface.')
            return

        self.prev_time = curr_time

        thrust_setpoint_list = msg.thrust
        self.thrust_setpoint = thrust_setpoint_list

        self.update_reference(dt)
        self.set_pwm()

    def pulse_width_in_bits(self, force):
        pulse_width_in_microseconds = numpy.interp(force, self.T100_thrust, self.T100_pulse_width)
        normalized_duty_cycle = pulse_width_in_microseconds/self.PERIOD_LENGTH_IN_MICROSECONDS
        return int(round(self.PWM_BITS_PER_PERIOD * normalized_duty_cycle))

    def update_reference(self, dt):
        rate_of_change = (self.thrust_setpoint - self.thrust_reference)/dt

        for i in range(self.num_thrusters):
            if rate_of_change[i] > self.max_rate:
                self.thrust_reference[i] += dt * self.max_rate
            elif rate_of_change[i] < -self.max_rate:
                self.thrust_reference[i] -= dt * self.max_rate
            else:
                self.thrust_reference[i] = self.thrust_setpoint[i]

    def set_pwm(self):
        pwm_state = [None]*self.num_thrusters
        for i in range(self.num_thrusters):
            pwm_state[i] = self.pulse_width_in_bits(self.thrust_reference[i])
            # pca9685.set_pwm(i, 0, pwm_state[i])

        # Publish outputs for debug
        pwm_msg = ThrusterPwm()
        pwm_msg.header.stamp = rospy.get_rostime()
        pwm_msg.pwm = pwm_state
        self.pub.publish(pwm_msg)

if __name__ == '__main__':
    try:
        motor_interface = MotorInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
