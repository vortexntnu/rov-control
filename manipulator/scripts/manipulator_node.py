#!/usr/bin/env python
# Ros node for controll of the manipulator arm


import RPi.GPIO as GPIO
import rospy
from vortex_msgs.msg import JoystickArmCommand
from vortex_msgs.msg import ArmState


class ManipulatorNode():

    grip_increment_value = 30
    base_increment_value = 1
    rot_increment_value = 15

    grip_max =  90
    grip_min =  0

    base_max =  180
    base_min =  0

    rot_max  =  240
    rot_min  =  0

    def __init__(self):

        rospy.init_node('manipulator_node', anonymous=True)

        self.state_publisher = rospy.Publisher('arm_state', ArmState, queue_size=10)
        self.ideal = ArmState()
        self.ideal_grip_angle = 80
        self.ideal_base_angle = 0
        self.ideal_rot_angle  = 0

        self.ideal_base1 = 0
        self.ideal_base2 = 0

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        # Control signal from pin 11, 13 and 15


        # pwmpin_grip     = 11
        # pwmpin_base1    = 15
        # pwmpin_base2    = 16
        # pwmpin_rot      = 13


        pwmpin_grip     = 11
        pwmpin_base1    = 15
        pwmpin_base2    = 16
        pwmpin_rot      = 13

        # GPIO.setup(pwmpin_grip, GPIO.OUT)
        # self.pwm_grip_control = GPIO.PWM(pwmpin_grip, 100)
        # self.pwm_grip_control.start(0)

        GPIO.setup(pwmpin_base1, GPIO.OUT)
        self.pwm_base1_control = GPIO.PWM(pwmpin_base1, 100)
        self.pwm_base1_control.start(0)

        GPIO.setup(pwmpin_base2, GPIO.OUT)
        self.pwm_base2_control = GPIO.PWM(pwmpin_base2, 100)
        self.pwm_base2_control.start(0)

        # GPIO.setup(pwmpin_rot, GPIO.OUT)
        # self.pwm_rot_control = GPIO.PWM(pwmpin_rot, 100)
        # self.pwm_rot_control.start(0)

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
        # Grip
        # grip_duty = (float(self.ideal_grip_angle) / 10.0) + 2.5
        # self.pwm_grip_control.ChangeDutyCycle(grip_duty)
        # print("grip duty")
        # print(grip_duty)

        # Base1
        base1_normalized = get_base1_offset(self.ideal_base_angle)
        base1_duty = (float(base1_normalized) /10.0) + 2.5
        self.pwm_base1_control.ChangeDutyCycle(base1_duty)
        # print("Base 1 duty")
        # print(base1_duty)

        # Base2
        base2_normalized = get_base2_offset(self.ideal_base_angle)
        base2_duty = (float(180 - self.ideal_base_angle + 18) /10.0) + 2.5
        self.pwm_base2_control.ChangeDutyCycle(base2_duty)
        # print("Base 2 duty")
        # print(base2_duty)

        # Rotation
        # rot_duty = (float(self.ideal_rot_angle) / 10.0) + 2.5
        # self.pwm_rot_control.ChangeDutyCycle(rot_duty)
        # print("rot duty")
        # print(rot_duty)


def to_range(minv, maxv, v):
    if (v < minv):
        return minv
    if (v > maxv):
        return maxv
    return v


base1_offset = 40
base2_offset = 18

def get_base1_offset(ideal):
    return ideal + base1_offset

def get_base2_offset(ideal):
    return ideal + base2_offset


if __name__ == '__main__':
    manipulator_node = ManipulatorNode()

