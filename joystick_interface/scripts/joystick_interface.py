#!/usr/bin/env python
import rospy
import vortex_msgs.msg
from sensor_msgs.msg import Joy

class JoystickInterfaceNode(object):
    def __init__(self):
        rospy.init_node('joystick_node')
        self.arm_msg = vortex_msgs.msg.JoystickArmCommand()
        self.motion_msg = vortex_msgs.msg.JoystickMotionCommand()
        self.sub = rospy.Subscriber('joy', Joy, self.callback, queue_size=10)
        self.pub_motion = rospy.Publisher('joystick_motion_command',
                                          vortex_msgs.msg.JoystickMotionCommand,
                                          queue_size=10)
        self.pub_arm = rospy.Publisher('joystick_arm_command',
                                       vortex_msgs.msg.JoystickArmCommand,
                                       queue_size=10)

        self.buttons_map = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'back',
                            'start', 'power', 'stick_button_left',
                            'stick_button_right']

        self.axes_map = ['horizontal_axis_left_stick',
                         'vertical_axis_left_stick', 'LT',
                         'horizontal_axis_right_stick',
                         'vertical_axis_right_stick', 'RT',
                         'dpad_horizontal', 'dpad_vertical']

    def callback(self, msg):
        buttons = {}
        axes = {}

        for i in range(len(msg.buttons)):
            buttons[self.buttons_map[i]] = msg.buttons[i]

        for j in range(len(msg.axes)):
            axes[self.axes_map[j]] = msg.axes[j]

        arm_msg_fields = {
            'base_up' : axes['dpad_vertical'],
            'base_down' : axes['dpad_vertical'] == -1,
            'rot_left' : axes['dpad_horizontal'],
            'rot_right' : axes['dpad_horizontal'] == -1,
            'grip_open' : buttons['B'],
            'grip_close' : buttons['A']
        }

        motion_msg_fields = {
            'forward' : -axes['vertical_axis_left_stick'],
            'right' : axes['horizontal_axis_left_stick'],
            'down' : axes['RT'] - axes['LT'],
            'tilt_up' : axes['vertical_axis_right_stick'],
            'turn_right' : axes['horizontal_axis_right_stick']
        }

        for field, value in arm_msg_fields.iteritems():
            setattr(self.arm_msg, field, value)

        for field, value in motion_msg_fields.iteritems():
            setattr(self.motion_msg, field, value)

        #Control mode is separate to avoid overwriting old mode
        if buttons['back']:
            self.motion_msg.control_mode = 1
        if buttons['start']:
            self.motion_msg.control_mode = 0

        self.pub_arm.publish(self.arm_msg)
        self.pub_motion.publish(self.motion_msg)

if __name__ == '__main__':
    try:
        joystick_node = JoystickInterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
