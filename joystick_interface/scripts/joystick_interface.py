#!/usr/bin/env python
import rospy
import vortex_msgs.msg
from sensor_msgs.msg import Joy

class JoystickInterfaceNode(object):
    def __init__(self):
        rospy.init_node('joystick_node')
        self.arm_cmd = vortex_msgs.msg.JoystickArmCommand()
        self.motion_cmd = vortex_msgs.msg.JoystickMotionCommand()
        self.sub = rospy.Subscriber('joy', Joy, self.callback, queue_size=10)
        self.pub_motion = rospy.Publisher('joystick_motion_command',
                                          vortex_msgs.msg.JoystickMotionCommand,
                                          queue_size=10)
        self.pub_arm = rospy.Publisher('joystick_arm_command',
                                       vortex_msgs.msg.JoystickArmCommand,
                                       queue_size=10)

    def callback(self, msg):
        button_mapping = {
            0: ('A', 'grip_close'),
            1: ('B', 'grip_open'),
            2: ('X', None),
            3: ('Y', None),
            4: ('LB', None),
            5: ('RB', None),
            6: ('back', 'control_mode'),
            7: ('start', 'control_mode'),
            8: ('power', None),
            9: ('stick_button_left', None),
            10: ('stick_button_right', None)
        }
        axes_mapping = {
            0: ('horizontal_axis_left_stick', 'right'),
            1: ('vertical_axis_left_stick', 'forward'),
            2: ('LT', None),
            3: ('horizontal_axis_right_stick', 'turn_right'),
            4: ('vertical_axis_right_stick', 'tilt_up'),
            5: ('RT', None),
            6: ('cross_horizontal', None),
            7: ('cross_vertical', None)
        }

if __name__ == '__main__':
    try:
        joystick_node = JoystickInterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
