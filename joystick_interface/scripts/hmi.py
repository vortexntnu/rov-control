#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from vortex_msgs.msg import Joystick

from vortex_msgs.msg import JoystickMotionCommand
from vortex_msgs.msg import JoystickArmCommand

XBOX_JOYSTICK_MAX = 32768.0
XBOX_TRIGGER_MAX  = 255.0

class HmiNode:
    def __init__(self):
        rospy.init_node("hmi_node", anonymous=False)

        self.motion_command = JoystickMotionCommand()
        self.arm_command = JoystickArmCommand()

        self.uranus_publisher = rospy.Publisher('joystick_motion_command', JoystickMotionCommand, queue_size=10)
        self.manipulator_publisher = rospy.Publisher('joystick_arm_command', JoystickArmCommand, queue_size=10)

        rospy.Subscriber('joystick', Joystick, self.joystick_callback)
        self.rate = rospy.Rate(10)

    def start_node(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def joystick_callback(self, joystick):
        # ROV movement
        self.motion_command.forward    = -joystick.strafe_Y                   / XBOX_JOYSTICK_MAX
        self.motion_command.right      =  joystick.strafe_X                   / XBOX_JOYSTICK_MAX
        self.motion_command.turn_right =  joystick.turn_X                     / XBOX_JOYSTICK_MAX
        self.motion_command.tilt_up    =  joystick.turn_Y                     / XBOX_JOYSTICK_MAX
        self.motion_command.down       = (joystick.descend - joystick.ascend) / XBOX_TRIGGER_MAX

        # Control mode
        if(joystick.free_roam):
            self.motion_command.control_mode = 0
        if(joystick.hold_position):
            self.motion_command.control_mode = 1
        if(joystick.depth_hold):
            self.motion_command.control_mode = 3

        self.motion_command.header.stamp = rospy.get_rostime()
        self.uranus_publisher.publish(self.motion_command)

        # Manipulator
        self.arm_command.grip_open        =       joystick.arm_grip_open
        self.arm_command.grip_close       =       joystick.arm_grip_close

        self.arm_command.base_up          =       joystick.arm_base_up
        self.arm_command.base_down        =       joystick.arm_base_down

        self.arm_command.rot_left         =       joystick.arm_rot_left
        self.arm_command.rot_right        =       joystick.arm_rot_right

        self.manipulator_publisher.publish(self.arm_command)

if __name__ == '__main__':
    try:
        print("joystick_interface: Launching node hmi.")
        hmi_node = HmiNode()
        hmi_node.start_node()
    except rospy.ROSInterruptException:
        pass
