#!/usr/bin/env python

from numpy import clip, interp
import rospy

from vortex_msgs.msg import Manipulator, Pwm
from stepper import Stepper

SERVO_PWM_PIN = rospy.get_param('/pwm/pins/claw_servo')
LOOKUP_POSITION = rospy.get_param('/servo/lookup/position')
LOOKUP_PULSE_WIDTH = rospy.get_param('/servo/lookup/pulse_width')

STEPPER_NUM_STEPS = rospy.get_param('/stepper/steps_per_rev')
STEPPER_RPM = rospy.get_param('/stepper/default_speed_rpm')

STEPPER_VALVE_PINS = rospy.get_param('/stepper/pins/valve')
STEPPER_VALVE_ENABLE_PIN = rospy.get_param('/stepper/pins/valve_enable')

STEPPER_AGAR_PINS = rospy.get_param('/stepper/pins/agar')
STEPPER_AGAR_ENABLE_PIN = rospy.get_param('/stepper/pins/agar_enable')


class ManipulatorInterface(object):
    def __init__(self):
        rospy.init_node('manipulator_interface', anonymous=False)
        self.pub = rospy.Publisher('pwm', Pwm, queue_size=10)
        self.sub = rospy.Subscriber('manipulator_command', Manipulator, self.callback)

        self.neutral_pulse_width = self.servo_position_to_microsecs(0)

        rospy.sleep(0.1)  # Initial set to zero seems to disappear without a short sleep here
        self.servo_set_to_zero()
        rospy.on_shutdown(self.shutdown)
        self.claw_direction = 0.0
        self.claw_position = 0.0  # 1 = open, -1 = closed
        self.claw_speed = 0.5

        try:
            self.valve_stepper = Stepper(STEPPER_NUM_STEPS, STEPPER_VALVE_PINS, STEPPER_VALVE_ENABLE_PIN)
            self.valve_stepper.set_speed(STEPPER_RPM)
            self.valve_direction = 0

            self.agar_stepper = Stepper(STEPPER_NUM_STEPS, STEPPER_AGAR_PINS, STEPPER_AGAR_ENABLE_PIN)
            self.agar_stepper.set_speed(STEPPER_RPM)
            self.agar_direction = 0
        except NameError:
            rospy.logfatal('Could not initialize stepper.py. Is /computer parameter set correctly?'
                           'Shutting down node...')
            rospy.signal_shutdown('')

        rospy.loginfo('Initialized.')
        self.spin()

    def spin(self):
        period = 60.0 / (STEPPER_NUM_STEPS * STEPPER_RPM)
        rate = rospy.Rate(1/period)
        prev_time = rospy.get_rostime()
        min_pwm_interval = rospy.Duration(0.1)

        while not rospy.is_shutdown():
            # Accumulate claw position
            self.claw_position += self.claw_speed * period * self.claw_direction
            # Saturate claw position to [-1, 1]
            self.claw_position = clip(self.claw_position, -1, 1)

            self.valve_stepper.step(self.valve_direction)
            self.agar_stepper.step(self.agar_direction)

            # Avoid too frequent PWM commands
            if (rospy.get_rostime() - prev_time) > min_pwm_interval:
                self.set_claw_pwm(self.claw_position)
                prev_time = rospy.get_rostime()

            rate.sleep()

    def servo_set_to_zero(self):
        msg = Pwm()
        msg.pins.append(SERVO_PWM_PIN)
        msg.positive_width_us.append(self.neutral_pulse_width)
        self.pub.publish(msg)
        rospy.loginfo("Setting servo position to zero")

    def shutdown(self):
        self.servo_set_to_zero()
        self.valve_stepper.shutdown()

    def callback(self, msg):
        if not self.healthy_message(msg):
            return

        self.claw_direction = msg.claw_direction
        self.valve_direction = msg.valve_direction
        self.agar_direction = msg.agar_direction

        if self.valve_direction == 0:
            self.valve_stepper.disable()
        else:
            self.valve_stepper.enable()

        if self.agar_direction == 0:
            self.agar_stepper.disable()
        else:
            self.agar_stepper.enable()

    def servo_position_to_microsecs(self, thrust):
        return interp(thrust, LOOKUP_POSITION, LOOKUP_PULSE_WIDTH)

    def set_claw_pwm(self, position):
        microsecs = self.servo_position_to_microsecs(position)

        msg = Pwm()
        msg.pins.append(SERVO_PWM_PIN)
        msg.positive_width_us.append(microsecs)

        self.pub.publish(msg)

    def healthy_message(self, msg):
        if abs(msg.claw_direction) > 1:
            rospy.logwarn_throttle(1, 'Claw position out of range. Ignoring message...')
            return False

        if abs(msg.valve_direction) > 1:
            rospy.logwarn_throttle(1, 'Valve spinner command out of range. Ignoring message...')
            return False

        if abs(msg.agar_direction) > 1:
            rospy.logwarn_throttle(1, 'Agar screwer command out of range. Ignoring message...')
            return False

        return True


if __name__ == '__main__':
    try:
        manipulator_interface = ManipulatorInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
