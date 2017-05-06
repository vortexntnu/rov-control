#!/usr/bin/env python

import numpy
import rospy

from vortex_msgs.msg import Manipulator, Pwm
from stepper import Stepper

PWM_COUNTER_MAX = rospy.get_param('/pwm/counter/max')
FREQUENCY = rospy.get_param('/pwm/frequency/set')
FREQUENCY_MEASURED = rospy.get_param('/pwm/frequency/measured')
SERVO_PWM_PIN = rospy.get_param('/pwm/pins/claw_servo')
LOOKUP_POSITION = rospy.get_param('/servo/lookup/position')
LOOKUP_PULSE_WIDTH = rospy.get_param('/servo/lookup/pulse_width')
PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0 / FREQUENCY_MEASURED

STEPPER_NUM_STEPS = rospy.get_param('/stepper/steps_per_rev')
STEPPER_VALVE_PINS = rospy.get_param('/stepper/pins/valve')
STEPPER_VALVE_RPM = rospy.get_param('/stepper/default_speed_rpm')


class ManipulatorInterface(object):
    def __init__(self):
        rospy.init_node('manipulator_interface', anonymous=False)
        self.pub = rospy.Publisher('pwm', Pwm, queue_size=10)
        self.sub = rospy.Subscriber('manipulator_command', Manipulator, self.callback)

        # TODO(mortenfyhn): Consider setting neutral to fully open instead
        self.neutral_pulse_width = self.microsecs_to_bits(self.servo_position_to_microsecs(0))

        self.valve_stepper = Stepper(STEPPER_NUM_STEPS, STEPPER_VALVE_PINS)
        self.valve_stepper.set_speed(STEPPER_VALVE_RPM)
        self.valve_direction = 0

        rospy.sleep(0.1) # Initial set to zero seems to disappear without a short sleep here
        self.servo_set_to_zero()
        rospy.on_shutdown(self.servo_set_to_zero)
        rospy.loginfo("Launching for %d Hz PWM", FREQUENCY)

        self.spin()

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.valve_stepper.step(self.valve_direction)
            rate.sleep()

    def servo_set_to_zero(self):
        msg = Pwm()
        msg.pins.append(SERVO_PWM_PIN)
        msg.on.append(0)
        msg.off.append(self.neutral_pulse_width)
        self.pub.publish(msg)
        rospy.loginfo("Set to zero")

    def callback(self, msg):
        if not self.healthy_message(msg):
            return

        self.set_claw_pwm(msg.claw_position)

        if msg.open_valve:
            self.valve_direction = 1
        elif msg.close_valve:
            self.valve_direction = -1
        else:
            self.valve_direction = 0

    def servo_position_to_microsecs(self, thrust):
        return numpy.interp(thrust, LOOKUP_POSITION, LOOKUP_PULSE_WIDTH)

    def microsecs_to_bits(self, microsecs):
        duty_cycle_normalized = microsecs / PERIOD_LENGTH_IN_MICROSECONDS
        return int(round(PWM_COUNTER_MAX * duty_cycle_normalized))

    def set_claw_pwm(self, position):
        microsecs = self.servo_position_to_microsecs(position)
        pwm_bits = self.microsecs_to_bits(microsecs)

        msg = Pwm()
        msg.pins.append(SERVO_PWM_PIN)
        msg.on.append(0)
        msg.off.append(pwm_bits)

        self.pub.publish(msg)

    def healthy_message(self, msg):
        if abs(msg.claw_position) > 1:
            rospy.logwarn_throttle(1, 'Claw position out of range. Ignoring message...')
            return False

        if msg.open_valve and msg.close_valve:
            rospy.logwarn_throttle(1, 'Cannot open and close valve at the same time! Ignoring message...')
            return False

        return True


if __name__ == '__main__':
    try:
        manipulator_interface = ManipulatorInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
