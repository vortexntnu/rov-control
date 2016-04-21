#!/usr/bin/env python
# Ros node for controll of the manipulator arm


import RPi.GPIO as GPIO
import rospy
from maelstrom_msgs.msg import Joystick


class ManipulatorNode():

    max_soft_value = 255
    arm_increment_value = 20

    def __init__(self):
        self.model = Joystick()

        rospy.init_node('manipulator_node', anonymous=True)

	self.grip_angle = 0
	self.base_angle = 0
	self.rot_angle = 0

	GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
	# Control signal from pin 11, 13 and 15
        pwmpin_grip, pwmpin_base, pwmpin_rot = 11, 13, 15
        GPIO.setup(pwmpin_grip, GPIO.OUT)
        GPIO.setup(pwmpin_base, GPIO.OUT)
        GPIO.setup(pwmpin_rot, GPIO.OUT)
        # create an object for PWM control at 100 Hz
        self.pwm_grip_control = GPIO.PWM(pwmpin_grip, 100)
        self.pwm_base_control = GPIO.PWM(pwmpin_base, 100)
        self.pwm_rot_control = GPIO.PWM(pwmpin_rot, 100)
        self.pwm_grip_control.start(0)
        self.pwm_base_control.start(0)
        self.pwm_rot_control.start(0)
        self.control_listener()

    def control_listener(self):

        def callback(data):
	    print data
            self.model = data
            self.update_manipulator()

        rospy.Subscriber("joystick", Joystick, callback)
        rospy.spin()

    def update_manipulator(self):
	# Grip
	self.grip_angle =int((self.model.descend / float(ManipulatorNode.max_soft_value)) * 180)
	grip_duty = float(self.grip_angle) / 10.0 + 2.5
        self.pwm_grip_control.ChangeDutyCycle(grip_duty)
	# Base
	self.base_angle = min(max(self.base_angle + ManipulatorNode.arm_increment_value * self.model.arm_base, 1), 179)
	base_duty = float(self.base_angle) / 10.0 + 2.5
        self.pwm_base_control.ChangeDutyCycle(base_duty)
	# Rotation
	self.rot_angle = min(max(self.rot_angle + ManipulatorNode.arm_increment_value * self.model.arm_rot, 1), 179)
	rot_duty = float(self.rot_angle) / 10.0 + 2.5
        self.pwm_rot_control.ChangeDutyCycle(rot_duty)
	print self.grip_angle, self.base_angle, self.rot_angle


if __name__ == '__main__':
    manipulator_node = ManipulatorNode()

