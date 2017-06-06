#!/usr/bin/env python

import numpy
import rospy

from vortex_msgs.msg import Manipulator, Pwm
from stepper import Stepper

FREQUENCY = rospy.get_param('/pwm/frequency/set')
SERVO_PWM_PIN = rospy.get_param('/pwm/pins/claw_servo')
LOOKUP_POSITION = rospy.get_param('/servo/lookup/position')
LOOKUP_PULSE_WIDTH = rospy.get_param('/servo/lookup/pulse_width')

STEPPER_NUM_STEPS = rospy.get_param('/stepper/steps_per_rev')
STEPPER_VALVE_PINS = rospy.get_param('/stepper/pins/valve')
STEPPER_VALVE_DISABLE_PIN = rospy.get_param('/stepper/pins/valve_disable')
STEPPER_VALVE_RPM = rospy.get_param('/stepper/default_speed_rpm')


class ManipulatorInterface(object):
    def __init__(self):
        rospy.init_node('manipulator_interface', anonymous=False)
        self.pub = rospy.Publisher('pwm', Pwm, queue_size=10)
        self.sub = rospy.Subscriber('manipulator_command', Manipulator, self.callback)

        self.neutral_pulse_width = self.servo_position_to_microsecs(0)

        rospy.sleep(0.1)  # Initial set to zero seems to disappear without a short sleep here
        self.servo_set_to_zero()
        rospy.on_shutdown(self.shutdown)
        self.claw_direction = 0.0  # 1 = open more, -1 = close more, 0 = do nothing
        self.claw_position = 0.0  # 1 = open, -1 = closed
        self.claw_speed = 0.5

        try:
            self.valve_stepper = Stepper(STEPPER_NUM_STEPS, STEPPER_VALVE_PINS, STEPPER_VALVE_DISABLE_PIN)
            self.valve_stepper.set_speed(STEPPER_VALVE_RPM)
            self.valve_direction = 0
        except NameError:
            rospy.logfatal('Could not initialize stepper.py. Is /computer parameter set correctly? Shutting down...')
            rospy.signal_shutdown('')

        self.spinning_valve_locked = False

        rospy.loginfo("Launching for %d Hz PWM", FREQUENCY)
        self.spin()

    def spin(self):
        period = 60.0 / (STEPPER_NUM_STEPS * STEPPER_VALVE_RPM)
        rate = rospy.Rate(1/period)
        while not rospy.is_shutdown():
            # Accumulate claw position
            self.claw_position += self.claw_speed * period * self.claw_direction
            # Saturate claw position to [-1, 1]
            if self.claw_position >= 1:
                self.claw_position = 1
            elif self.claw_position <= -1:
                self.claw_position = -1

            self.set_claw_pwm(self.claw_position)
            self.valve_stepper.step(self.valve_direction)
            rate.sleep()

    def servo_set_to_zero(self):
        msg = Pwm()
        msg.pins.append(SERVO_PWM_PIN)
        msg.microseconds.append(self.neutral_pulse_width)
        self.pub.publish(msg)
        rospy.loginfo("Set to zero")

    def shutdown(self):
        self.servo_set_to_zero()
        self.valve_stepper.shutdown()

    def callback(self, msg):
        if not self.healthy_message(msg):
            return

        self.claw_direction = msg.claw_position
        self.valve_direction = msg.valve_direction

        if self.valve_direction == 0:
            self.valve_stepper.disable()
        else:
            self.valve_stepper.enable()

    def servo_position_to_microsecs(self, thrust):
        return numpy.interp(thrust, LOOKUP_POSITION, LOOKUP_PULSE_WIDTH)

    def set_claw_pwm(self, position):
        microsecs = self.servo_position_to_microsecs(position)

        msg = Pwm()
        msg.pins.append(SERVO_PWM_PIN)
        msg.microseconds.append(microsecs)

        self.pub.publish(msg)

    def healthy_message(self, msg):
        if abs(msg.claw_position) > 1:
            rospy.logwarn_throttle(1, 'Claw position out of range. Ignoring message...')
            return False

        if abs(msg.valve_direction) > 1:
            rospy.logwarn_throttle(1, 'Valve spinner command out of range. Ignoring message...')
            return False

        return True


if __name__ == '__main__':
    try:
        manipulator_interface = ManipulatorInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
