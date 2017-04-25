#!/usr/bin/env python
import rospy
import vortex_msgs.msg
from sensor_msgs.msg import Joy

NUM_CONTROL_MODES = 4

class JoystickInterfaceNode(object):
    def __init__(self):
        rospy.init_node('joystick_node')
        self.arm_msg = vortex_msgs.msg.JoystickArmCommand()
        self.motion_msg = vortex_msgs.msg.PropulsionCommand()
        self.sub = rospy.Subscriber('joy', Joy, self.callback, queue_size=10)
        self.pub_motion = rospy.Publisher('propulsion_command',
                                          vortex_msgs.msg.PropulsionCommand,
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

        arm_msg_fields = [
            axes['dpad_vertical'],
            -axes['dpad_horizontal'],
            (buttons['A'] - buttons['B'])
        ]

        motion_msg_motion = [
            axes['vertical_axis_left_stick'],
            -axes['horizontal_axis_left_stick'],
            (buttons['RB'] - buttons['LB']),
            (axes['RT'] - axes['LT'])/2,
            -axes['vertical_axis_right_stick'],
            -axes['horizontal_axis_right_stick']
        ]

        motion_msg_control_mode = [
            (buttons ['back'] == 1),
            (buttons ['start'] == 1)
        ]
        self.arm_msg.manipulator = [None]*3
        for i in range(3):
            self.arm_msg.manipulator[i] = arm_msg_fields[i]

        self.motion_msg.motion = [None]*6
        for i in range(6):
            self.motion_msg.motion[i] = motion_msg_motion[i]

        # self.motion_msg.control_mode = [None]*2
        # for i in range(2):
        #     self.motion_msg.control_mode[i] = motion_msg_control_mode[i]
        self.motion_msg.control_mode = [False]*NUM_CONTROL_MODES
        self.motion_msg.control_mode[0] = True

        self.motion_msg.header.stamp = rospy.get_rostime()

        self.pub_arm.publish(self.arm_msg)
        self.pub_motion.publish(self.motion_msg)

if __name__ == '__main__':
    try:
        joystick_node = JoystickInterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
