#!/usr/bin/env python

from math import isnan, isinf
from numpy import interp
import rospy

from vortex_msgs.msg import Float64ArrayStamped, Pwm
from thruster_interface.srv import ThrustersEnable, ThrustersEnableResponse

THRUST_RANGE_LIMIT = 100

NUM_THRUSTERS = rospy.get_param('/propulsion/thrusters/num')
THRUST_OFFSET = rospy.get_param('/thrusters/offset')
LOOKUP_THRUST = rospy.get_param('/thrusters/characteristics/thrust')
LOOKUP_PULSE_WIDTH = rospy.get_param('/thrusters/characteristics/pulse_width')
THRUSTERS_CONNECTED = rospy.get_param('/thruster_interface/thrusters_connected')
THRUSTER_PWM_PINS = rospy.get_param('/pwm/pins/thrusters')


class ThrusterInterface(object):
    def __init__(self):
        rospy.init_node('thruster_interface', anonymous=False)
        self.pub_pwm = rospy.Publisher('pwm', Pwm, queue_size=10)
        self.sub = rospy.Subscriber('thruster_forces', Float64ArrayStamped, self.callback)
        self.srv = rospy.Service('/thruster_interface/thrusters_enable', ThrustersEnable, self.handle_thrusters_enable)

        self.thrusters_enabled = True

        self.output_to_zero()
        rospy.on_shutdown(self.output_to_zero)
        rospy.loginfo('Initialized with offset:\n\t{0}.'.format(THRUST_OFFSET))

    def output_to_zero(self):
        neutral_pulse_width = self.thrust_to_microsecs(0)
        if THRUSTERS_CONNECTED and self.thrusters_enabled:
            pwm_msg = Pwm()
            for i in range(NUM_THRUSTERS):
                pwm_msg.pins.append(THRUSTER_PWM_PINS[i])
                pwm_msg.positive_width_us.append(neutral_pulse_width)
            self.pub_pwm.publish(pwm_msg)

    def callback(self, msg):
        if not self.healthy_message(msg):
            return
        thrust = list(msg.data)

        microsecs = [None] * NUM_THRUSTERS
        pwm_msg = Pwm()

        for i in range(NUM_THRUSTERS):
            microsecs[i] = self.thrust_to_microsecs(thrust[i] + THRUST_OFFSET[i])
            pwm_msg.pins.append(THRUSTER_PWM_PINS[i])
            pwm_msg.positive_width_us.append(microsecs[i])
        if THRUSTERS_CONNECTED and self.thrusters_enabled:
            self.pub_pwm.publish(pwm_msg)

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
        return interp(thrust, LOOKUP_THRUST, LOOKUP_PULSE_WIDTH)

    def healthy_message(self, msg):
        if (len(msg.data) != NUM_THRUSTERS):
            rospy.logwarn_throttle(10, 'Wrong number of thrusters, ignoring...')
            return False

        for t in msg.data:
            if isnan(t) or isinf(t) or (abs(t) > THRUST_RANGE_LIMIT):
                rospy.logwarn_throttle(10, 'Message out of range, ignoring...')
                return False
        return True


if __name__ == '__main__':
    try:
        thruster_interface = ThrusterInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
