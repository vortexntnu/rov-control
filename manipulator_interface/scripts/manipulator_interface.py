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
STEPPER_RPM = rospy.get_param('/stepper/default_speed_rpm')

STEPPER_VALVE_PINS = rospy.get_param('/stepper/pins/valve')
STEPPER_VALVE_DISABLE_PIN = rospy.get_param('/stepper/pins/valve_disable')

STEPPER_SCREW_PINS = rospy.get_param('/stepper/pins/screw')
STEPPER_SCREW_DISABLE_PIN = rospy.get_param('/stepper/pins/screw_disable')


class ManipulatorInterface(object):
    def __init__(self):
        rospy.init_node('manipulator_interface', anonymous=False)
        self.pub = rospy.Publisher('pwm', Pwm, queue_size=10)
        self.sub = rospy.Subscriber('manipulator_command', Manipulator, self.callback)

        self.neutral_pulse_width = self.microsecs_to_bits(self.servo_position_to_microsecs(0))

        rospy.sleep(0.1)  # Initial set to zero seems to disappear without a short sleep here
        self.servo_set_to_zero()
        rospy.on_shutdown(self.shutdown)
        self.claw_direction = 0.0
        self.claw_position = 0.0  # 1 = open, -1 = closed
        self.claw_speed = 0.5

        try:
            self.valve_stepper = Stepper(STEPPER_NUM_STEPS, STEPPER_VALVE_PINS, STEPPER_VALVE_DISABLE_PIN)
            self.valve_stepper.set_speed(STEPPER_RPM)
            self.valve_direction = 0

            self.screw_stepper = Stepper(STEPPER_NUM_STEPS, STEPPER_SCREW_PINS, STEPPER_SCREW_DISABLE_PIN)
            self.screw_stepper.set_speed(STEPPER_RPM)
            self.screw_direction = 0
        except NameError:
            rospy.logfatal('Could not initialize stepper.py. Is /computer parameter set correctly?'
                           'Shutting down node...')
            rospy.signal_shutdown('')

        rospy.loginfo("Launching for %d Hz PWM", FREQUENCY)
        self.spin()

    def spin(self):
        period = 60.0 / (STEPPER_NUM_STEPS * STEPPER_RPM)
        rate = rospy.Rate(1/period)
        while not rospy.is_shutdown():
            # Accumulate claw position
            self.claw_position += self.claw_speed * period * self.claw_direction
            # Saturate claw position to [-1, 1]
            self.claw_position = numpy.clip(self.claw_position, -1, 1)

            self.set_claw_pwm(self.claw_position)
            self.valve_stepper.step(self.valve_direction)
            self.screw_stepper.step(self.screw_direction)
            rate.sleep()

    def servo_set_to_zero(self):
        msg = Pwm()
        msg.pins.append(SERVO_PWM_PIN)
        msg.on.append(0)
        msg.off.append(self.neutral_pulse_width)
        self.pub.publish(msg)
        rospy.loginfo("Set to zero")

    def shutdown(self):
        self.servo_set_to_zero()
        self.valve_stepper.shutdown()

    def callback(self, msg):
        if not self.healthy_message(msg):
            return

        self.claw_direction = msg.claw_direction
        self.valve_direction = msg.valve_direction
        self.screw_direction = msg.screw_direction

        if self.valve_direction == 0:
            self.valve_stepper.disable()
        else:
            self.valve_stepper.enable()

        if self.screw_direction == 0:
            self.screw_stepper.disable()
        else:
            self.screw_stepper.enable()

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
        if abs(msg.claw_direction) > 1:
            rospy.logwarn_throttle(1, 'Claw position out of range. Ignoring message...')
            return False

        if abs(msg.valve_direction) > 1:
            rospy.logwarn_throttle(1, 'Valve spinner command out of range. Ignoring message...')
            return False

        if abs(msg.screw_direction) > 1:
            rospy.logwarn_throttle(1, 'Screwer command out of range. Ignoring message...')
            return False

        return True


if __name__ == '__main__':
    try:
        manipulator_interface = ManipulatorInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
