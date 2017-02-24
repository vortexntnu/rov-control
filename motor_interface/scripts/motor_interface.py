#!/usr/bin/env python

import rospy
import numpy
import math
import Adafruit_PCA9685
from vortex_msgs.msg import Float64ArrayStamped

class MotorInterface(object):
    def __init__(self):
        rospy.init_node('motor_interface', anonymous=False)
        self.pub = rospy.Publisher('debug/thruster_pwm', Float64ArrayStamped, queue_size=10)
        self.sub = rospy.Subscriber('thruster_forces', Float64ArrayStamped, self.callback)

        self.PWM_BITS_PER_PERIOD           = 4096.0 # 12 bit PWM
        self.FREQUENCY                     = 249    # Max 400 Hz
        self.FREQUENCY_MEASURED            = 251.2  # Use this for better precision
        self.PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0/self.FREQUENCY_MEASURED
        self.THRUST_RANGE_LIMIT            = 100

        self.ENABLE_RATE_LIMITER = False

        self.T100_thrust      = rospy.get_param('/thrusters/characteristics/thrust')
        self.T100_pulse_width = rospy.get_param('/thrusters/characteristics/pulse_width')
        self.num_thrusters    = rospy.get_param('/propulsion/thrusters/num')
        self.max_rate         = rospy.get_param('/thrusters/rate_of_change/max')
        self.motor_connection_enabled = rospy.get_param('/motor_interface/motor_connection_enabled')
        self.rate_limiting_enabled    = rospy.get_param('/motor_interface/rate_limiting_enabled')
        self.prev_time = rospy.get_rostime()
        self.is_initialized = False

        # The setpoint is the desired value (input)
        self.thrust_setpoint  = numpy.zeros(self.num_thrusters)
        # The reference is the output value (rate limited)
        self.thrust_reference = numpy.zeros(self.num_thrusters)

        # Initialize the PCA9685 using the default address (0x40)
        if (self.motor_connection_enabled):
            self.pca9685 = Adafruit_PCA9685.PCA9685()
            self.pca9685.set_pwm_freq(self.FREQUENCY)

        # Initialize outputs to zero newton
        neutral_pulse_width = self.microsecs_to_bits(self.thrust_to_microsecs(0))
        if (self.motor_connection_enabled):
            for i in range(self.num_thrusters):
                self.pca9685.set_pwm(i, 0, neutral_pulse_width)

        print 'Launching node motor_interface at', self.FREQUENCY, 'Hz'

    def callback(self, msg):
        if not self.healthy_message(msg):
            return

        if not self.is_initialized:
            self.prev_time = msg.header.stamp
            self.is_initialized = True
            rospy.loginfo('Initialized motor_interface')
            return

        curr_time = msg.header.stamp
        dt = (curr_time - self.prev_time).to_sec()
        if (dt <= 0) and self.rate_limiting_enabled:
            rospy.logwarn_throttle(1, 'Motor interface: Zero time difference between messages, ignoring...')
            return

        self.prev_time = curr_time

        thrust_setpoint_list = msg.data
        self.thrust_setpoint = thrust_setpoint_list

        self.update_reference(dt)
        self.set_pwm()

    def thrust_to_microsecs(self, thrust):
        return numpy.interp(thrust, self.T100_thrust, self.T100_pulse_width)

    def microsecs_to_bits(self, microsecs):
        duty_cycle_normalized = microsecs / self.PERIOD_LENGTH_IN_MICROSECONDS
        return int(round(self.PWM_BITS_PER_PERIOD * duty_cycle_normalized))

    def update_reference(self, dt):
        if self.rate_limiting_enabled:
            rate_of_change = (self.thrust_setpoint - self.thrust_reference)/dt
            for i in range(self.num_thrusters):
                if rate_of_change[i] > self.max_rate:
                    self.thrust_reference[i] += dt * self.max_rate
                elif rate_of_change[i] < -self.max_rate:
                    self.thrust_reference[i] -= dt * self.max_rate
                else:
                    self.thrust_reference[i] = self.thrust_setpoint[i]
        else:
            self.thrust_reference = self.thrust_setpoint

    def set_pwm(self):
        microsecs = [None]*self.num_thrusters
        for i in range(self.num_thrusters):
            microsecs[i] = self.thrust_to_microsecs(self.thrust_reference[i])
            pwm_bits = self.microsecs_to_bits(microsecs[i])
            if (self.motor_connection_enabled):
                self.pca9685.set_pwm(i, 0, pwm_bits)

        # Publish outputs for debug
        debug_msg = Float64ArrayStamped()
        debug_msg.header.stamp = rospy.get_rostime()
        debug_msg.data = microsecs
        self.pub.publish(debug_msg)

    def healthy_message(self, msg):
        if (len(msg.data) != self.num_thrusters):
            rospy.logwarn_throttle(1, 'Motor interface: Wrong number of thrusters, ignoring...')
            return False

        for t in msg.data:
            if math.isnan(t) or math.isinf(t) or (abs(t) > self.THRUST_RANGE_LIMIT):
                rospy.logwarn_throttle(1, 'Motor interface: Message out of range, ignoring...')
                return False
        return True



if __name__ == '__main__':
    try:
        motor_interface = MotorInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
