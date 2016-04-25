#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from maelstrom_msgs.msg import Joystick
from maelstrom_msgs.msg import JoystickMotionCommand

XBOX_JOYSTICK_RANGE = 32768.0
XBOX_TRIGGER_RANGE  = 255.0

class HmiNode:

    def __init__(self):
        rospy.init_node("hmi_node", anonymous=True)

        self.directional_input = JoystickMotionCommand()

        self.uranus_publisher = rospy.Publisher('joystick_motion_command', JoystickMotionCommand, queue_size=10)

        rospy.Subscriber('joystick', Joystick, self.joystick_callback)
        self.rate = rospy.Rate(10)


    def start_node(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def joystick_callback(self, joystick):
        
        self.directional_input.forward      =       -joystick.strafe_Y                      /   XBOX_JOYSTICK_RANGE
        self.directional_input.right        =       joystick.strafe_X                       /   XBOX_JOYSTICK_RANGE
        self.directional_input.turn_right   =       joystick.turn_X                         /   XBOX_JOYSTICK_RANGE
        self.directional_input.tilt_up      =       joystick.turn_Y                         /   XBOX_JOYSTICK_RANGE
        self.directional_input.down         =       (joystick.descend - joystick.ascend)    /   XBOX_TRIGGER_RANGE

        if(joystick.free_roam): 
            self.directional_input.control_mode = 0

        if(joystick.hold_position): 
            self.directional_input.control_mode = 1

        self.uranus_publisher.publish(self.directional_input)

if __name__ == '__main__': 
    try:
        print("ready to fuck up")
        hmi_node = HmiNode()
        hmi_node.start_node()
    except rospy.ROSInterruptException:
        pass


  
