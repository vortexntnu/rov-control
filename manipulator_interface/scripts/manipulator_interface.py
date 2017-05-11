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

        rospy.sleep(0.1)  # Initial set to zero seems to disappear without a short sleep here
        self.servo_set_to_zero()
        rospy.on_shutdown(self.shutdown)
	self.claw_state = -1 #-1 = closed, 1 = open.

        try:
            self.valve_stepper = Stepper(STEPPER_NUM_STEPS, STEPPER_VALVE_PINS)
            self.valve_stepper.set_speed(STEPPER_VALVE_RPM)
            self.valve_direction = 0
        except NameError:
            rospy.logfatal('Could not initialize stepper.py. Is /computer parameter set correctly? Shutting down...')
            rospy.signal_shutdown('')

        self.spinning_valve_locked = False

        rospy.loginfo("Launching for %d Hz PWM", FREQUENCY)
        self.spin()

    def spin(self):
        rate = rospy.Rate((STEPPER_NUM_STEPS * STEPPER_VALVE_RPM) / 60)
        while not rospy.is_shutdown():
            #self.valve_stepper.step_now(self.valve_direction)
	    self.set_claw_pwm(self.claw_state)
            self.valve_stepper.step(self.valve_direction)
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

	if msg.claw_position == 1:
	    self.claw_state = 1
	elif msg.claw_position == -1:
	    self.claw_state = -1

#        self.set_claw_pwm(msg.claw_position)
        self.valve_direction = msg.valve_direction

#        if (not self.spinning_valve_locked) and (self.valve_direction != 0):
#            rospy.loginfo("acquiring valve lock")
#            self.spinning_valve_locked = True
#            self.valve_stepper.step(self.valve_direction * 100)
#            rospy.loginfo("releasing valve lock")
#            self.spinning_valve_locked = False
#        else:
#            rospy.loginfo("failed to acquire lock")

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
