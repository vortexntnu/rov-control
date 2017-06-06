#!/usr/bin/env python

import Adafruit_PCA9685
import rospy
from vortex_msgs.msg import Pwm

# Constants
PWM_BITS_PER_PERIOD = rospy.get_param('/pwm/counter/max')
FREQUENCY = rospy.get_param('/pwm/frequency/set')
FREQUENCY_MEASURED = rospy.get_param('/pwm/frequency/measured')
PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0 / FREQUENCY_MEASURED
PWM_ON = 0 # Start of duty cycle


class Pca9685InterfaceNode(object):
    def __init__(self):
        rospy.init_node('pwm_node')
        self.pca9685 = Adafruit_PCA9685.PCA9685()
        self.pca9685.set_pwm_freq(FREQUENCY)
        self.pca9685.set_all_pwm(0, 0)

        self.sub = rospy.Subscriber('pwm', Pwm, self.callback, queue_size=10)
        rospy.loginfo("Ready for PWM messages")

    def callback(self, msg):
        if len(msg.pins) == len(msg.on) == len(msg.off):
            for i in range(len(msg.pins)):
                self.pca9685.set_pwm(msg.pins[i], PWM_ON, self.microsecs_to_bits(msg.off[i]))

    def microsecs_to_bits(self, microsecs):
        duty_cycle_normalized = microsecs / PERIOD_LENGTH_IN_MICROSECONDS
        return int(round(PWM_BITS_PER_PERIOD * duty_cycle_normalized))


if __name__ == '__main__':
    try:
        pwm_node = Pca9685InterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
