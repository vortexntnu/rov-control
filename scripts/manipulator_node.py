#!/usr/bin/env python
# Ros node for controll of the manipulator arm


import RPi.GPIO as GPIO
import rospy
from maelstrom_msgs.msg import JoystickArmCommand
from manipulator.msg import ArmState


class ManipulatorNode():

    max_soft_value = 255
    arm_increment_value = 20

    grip_increment_value = 10
    base_increment_value = 10
    rot_increment_value = 10

    grip_max =  100
    grip_min =  0

    base_max =  180
    base_min =  0

    rot_max  =  180
    rot_min  =  0

    def __init__(self):

        rospy.init_node('manipulator_node', anonymous=True)

        self.state_publisher = rospy.Publisher('arm_state', ArmState, queue_size=10)
        self.ideal = ArmState()

	self.ideal_grip_angle = 0
	self.ideal_base_angle = 0
	self.ideal_rot_angle  = 0

	GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
	# Control signal from pin 11, 13 and 15
        pwmpin_grip, pwmpin_base1, pwmpin_base2, pwmpin_rot = 11, 15, 16, 13
        GPIO.setup(pwmpin_grip, GPIO.OUT)
        GPIO.setup(pwmpin_base1, GPIO.OUT)
        GPIO.setup(pwmpin_base2, GPIO.OUT)
        GPIO.setup(pwmpin_rot, GPIO.OUT)
        # create an object for PWM control at 100 Hz
        self.pwm_grip_control = GPIO.PWM(pwmpin_grip, 100)
        self.pwm_base1_control = GPIO.PWM(pwmpin_base1, 100)
        self.pwm_base2_control = GPIO.PWM(pwmpin_base2, 100)
        self.pwm_rot_control = GPIO.PWM(pwmpin_rot, 100)
        self.pwm_grip_control.start(0)
        self.pwm_base1_control.start(0)
        self.pwm_base2_control.start(0)
        self.pwm_rot_control.start(0)
        self.control_listener()

    def control_listener(self):

        def callback(data):
	    print data

            self.ideal_base_angle = self.ideal_base_angle + (ManipulatorNode.base_increment_value * (data.base_up   + -data.base_down ))
            self.ideal_rot_angle  = self.ideal_rot_angle  + (ManipulatorNode.rot_increment_value  * (data.rot_left  + -data.rot_right ))
            self.ideal_grip_angle = self.ideal_grip_angle + (ManipulatorNode.grip_increment_value * (data.grip_open + -data.grip_close))

            self.ideal_base_angle = to_range(ManipulatorNode.base_min, ManipulatorNode.base_max, self.ideal_base_angle)
            self.ideal_rot_angle  = to_range(ManipulatorNode.rot_min,  ManipulatorNode.rot_max,  self.ideal_rot_angle )
            self.ideal_grip_angle = to_range(ManipulatorNode.grip_min, ManipulatorNode.grip_max, self.ideal_grip_angle)

            self.ideal.ideal_base = self.ideal_base_angle 
            self.ideal.ideal_rot  = self.ideal_rot_angle
            self.ideal.ideal_grip = self.ideal_grip_angle

            self.state_publisher.publish(self.ideal)

            self.update_manipulator()

        rospy.Subscriber("joystick_arm_command", JoystickArmCommand, callback)
        rospy.spin()

    def update_manipulator(self):
        pass


	# Grip
	# self.grip_angle =int((self.model.descend / float(ManipulatorNode.max_soft_value)) * 180)
	# grip_duty = float(self.grip_angle) / 10.0 + 2.5
        # self.pwm_grip_control.ChangeDutyCycle(grip_duty)
        grip_duty = (float(self.ideal_grip_angle) / 10.0) + 2.5
        self.pwm_grip_control.ChangeDutyCycle(grip_duty)


	# # Base
	# self.base1_angle = min(max(self.base1_angle + ManipulatorNode.arm_increment_value * self.model.arm_base, 1), 179)
	# self.base2_angle = 180 - self.base1_angle
	# base1_duty = float(self.base1_angle) / 10.0 + 2.5
	# base2_duty = float(self.base2_angle) / 10.0 + 2.5
        # self.pwm_base1_control.ChangeDutyCycle(base1_duty)
        # self.pwm_base2_control.ChangeDutyCycle(base2_duty)
        base1_duty = (float(      self.ideal_base_angle) /10.0) + 2.5
        base2_duty = (float(180 - self.ideal_base_angle) /10.0) + 2.5
        self.pwm_base1_control.ChangeDutyCycle(base1_duty)
        self.pwm_base2_control.ChangeDutyCycle(base2_duty)


	# # Rotation
	# self.rot_angle = min(max(self.rot_angle + ManipulatorNode.arm_increment_value * self.model.arm_rot, 1), 179)
	# rot_duty = float(self.rot_angle) / 10.0 + 2.5
        # self.pwm_rot_control.ChangeDutyCycle(rot_duty)
	# print self.grip_angle, self.base1_angle, self.base2_angle, self.rot_angle
        rot_duty = (float(self.ideal_rot_angle) / 10.0) + 2.5
        self.pwm_rot_control.ChangeDutyCycle(rot_duty)



def to_range(minv, maxv, v):
    if (v < minv): 
        return minv
    if (v > maxv):
        return maxv
    return v

if __name__ == '__main__':
    manipulator_node = ManipulatorNode()

