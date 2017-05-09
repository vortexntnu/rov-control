#!/usr/bin/env python

import math
import numpy
import rospy

from vortex_msgs.msg import Float64ArrayStamped, Pwm
from thruster_interface.srv import ThrustersEnable, ThrustersEnableResponse

# Constants
PWM_BITS_PER_PERIOD = 4096.0  # 12 bit PWM
FREQUENCY = rospy.get_param('/pwm/frequency/set')
FREQUENCY_MEASURED = rospy.get_param('/pwm/frequency/measured')
PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0 / FREQUENCY_MEASURED
THRUST_RANGE_LIMIT = 100

LOOKUP_THRUST = rospy.get_param('/thrusters/characteristics/thrust')
LOOKUP_PULSE_WIDTH = rospy.get_param('/thrusters/characteristics/pulse_width')
NUM_THRUSTERS = rospy.get_param('/propulsion/thrusters/num')
MAX_RATE = rospy.get_param('/thrusters/rate_of_change/max')
RATE_LIMITING_ENABLED = rospy.get_param('/thruster_interface/rate_limiting_enabled')
THRUSTERS_CONNECTED = rospy.get_param('/thruster_interface/thrusters_connected')
THRUSTER_PWM_PINS = rospy.get_param('/pwm/pins/thrusters')


class ThrusterInterface(object):
    def __init__(self):
        rospy.init_node('thruster_interface', anonymous=False)
        self.pub = rospy.Publisher('debug/thruster_pwm', Float64ArrayStamped, queue_size=10)
        self.pub_pwm = rospy.Publisher('pwm', Pwm, queue_size=10)
        self.sub = rospy.Subscriber('thruster_forces', Float64ArrayStamped, self.callback)
        self.srv = rospy.Service('/thruster_interface/thrusters_enable', ThrustersEnable, self.handle_thrusters_enable)

        self.prev_time = rospy.get_rostime()
        self.is_initialized = False

        # The setpoint is the desired value (input)
        self.thrust_setpoint = numpy.zeros(NUM_THRUSTERS)
        # The reference is the output value (rate limited)
        self.thrust_reference = numpy.zeros(NUM_THRUSTERS)

        self.thrusters_enabled = True

        self.output_to_zero()
        rospy.on_shutdown(self.output_to_zero)
        rospy.loginfo("Launching at %d Hz", FREQUENCY)

    def output_to_zero(self):
        neutral_pulse_width = self.microsecs_to_bits(self.thrust_to_microsecs(0))
        if THRUSTERS_CONNECTED and self.thrusters_enabled:
            pwm_msg = Pwm()
            for i in range(NUM_THRUSTERS):
                pwm_msg.pins.append(THRUSTER_PWM_PINS[i])
                pwm_msg.on.append(0)
                pwm_msg.off.append(neutral_pulse_width)
            self.pub_pwm.publish(pwm_msg)

    def callback(self, msg):
        if not self.healthy_message(msg):
            return

        if not self.is_initialized:
            self.prev_time = msg.header.stamp
            self.is_initialized = True
            rospy.loginfo('Successfully initialized.')
            return

        curr_time = msg.header.stamp
        dt = (curr_time - self.prev_time).to_sec()
        if (dt <= 0) and RATE_LIMITING_ENABLED:
            rospy.logwarn_throttle(10, 'Zero time difference between messages, ignoring...')
            return

        self.prev_time = curr_time

        thrust_setpoint_list = msg.data
        self.thrust_setpoint = thrust_setpoint_list

        self.update_reference(dt)
        self.set_pwm()

    def handle_thrusters_enable(self, req):
        if req.thrusters_enable:
            rospy.loginfo('Enabling thrusters')
            self.thrusters_enabled = True
        else:
            rospy.loginfo('Disabling thrusters')
            self.output_to_zero()
            self.thrusters_enabled = False
        return ThrustersEnableResponse()

    def thrust_to_microsecs(self, thrust):
        return numpy.interp(thrust, LOOKUP_THRUST, LOOKUP_PULSE_WIDTH)

    def microsecs_to_bits(self, microsecs):
        duty_cycle_normalized = microsecs / PERIOD_LENGTH_IN_MICROSECONDS
        return int(round(PWM_BITS_PER_PERIOD * duty_cycle_normalized))

    def update_reference(self, dt):
        if RATE_LIMITING_ENABLED:
            rate_of_change = (self.thrust_setpoint - self.thrust_reference) / dt
            for i in range(NUM_THRUSTERS):
                if rate_of_change[i] > MAX_RATE:
                    self.thrust_reference[i] += dt * MAX_RATE
                elif rate_of_change[i] < -MAX_RATE:
                    self.thrust_reference[i] -= dt * MAX_RATE
                else:
                    self.thrust_reference[i] = self.thrust_setpoint[i]
        else:
            self.thrust_reference = self.thrust_setpoint

    def set_pwm(self):
        microsecs = [None] * NUM_THRUSTERS
        pwm_msg = Pwm()
        for i in range(NUM_THRUSTERS):
            microsecs[i] = self.thrust_to_microsecs(self.thrust_reference[i])
            pwm_bits = self.microsecs_to_bits(microsecs[i])
            pwm_msg.pins.append(THRUSTER_PWM_PINS[i])
            pwm_msg.on.append(0)
            pwm_msg.off.append(pwm_bits)
        if THRUSTERS_CONNECTED and self.thrusters_enabled:
            self.pub_pwm.publish(pwm_msg)

        # Publish outputs for debug
        debug_msg = Float64ArrayStamped()
        debug_msg.header.stamp = rospy.get_rostime()
        debug_msg.data = microsecs
        self.pub.publish(debug_msg)

    def healthy_message(self, msg):
        if (len(msg.data) != NUM_THRUSTERS):
            rospy.logwarn_throttle(10, 'Wrong number of thrusters, ignoring...')
            return False

        for t in msg.data:
            if math.isnan(t) or math.isinf(t) or (abs(t) > THRUST_RANGE_LIMIT):
                rospy.logwarn_throttle(10, 'Message out of range, ignoring...')

                return False
        return True


if __name__ == '__main__':
    try:
        thruster_interface = ThrusterInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
