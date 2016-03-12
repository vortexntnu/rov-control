#!/usr/bin/env python

import glob
from evdev import InputDevice
import rospy
import threading
from joystick.msg import Joystick
# from uranus_dp.msg import JoystickUranus
import xctrl
import subprocess
import os
import time



class JoystickNode:

    def __init__(self):
        self.model = Joystick()
        rospy.init_node('joystick_node')
        self.pub = rospy.Publisher('Joystick', Joystick, queue_size=10)
        self.rate = rospy.Rate(10)
    '''
        # to test the arduino
        while not rospy.is_shutdown():
            self.pub.publish("dadada")
            self.rate.sleep()
    '''

    def start_node(self):
        device_type, self.device = self.init_device()
        if device_type == 'xbox':
            # start reading xbox controller signals into control msg
            input_thread = threading.Thread(target=xctrl.read_ps3_into_control_msg, args=(self.device, self.model))
            input_thread.daemon = True
            input_thread.start()

        if device_type == 'ps3':
            xctrl.read_ps3_into_control_msg(self.device, self.model)

        while not rospy.is_shutdown():
            rospy.loginfo(self.model)
            self.pub.publish(self.model)
            self.rate.sleep()

    def init_device(self):
        os.system("pkill -f -9 xboxdrv")
        xboxdrv_process = subprocess.Popen(['xboxdrv', '--detach-kernel-driver', '-s'])
        time.sleep(1)
        devices = glob.glob('/dev/input/event*')
        for device_path in devices:
            print("trying device named " + device_path)
            try:
                device = InputDevice(device_path)
                print device.name.lower()
                # TODO device name of ps3 contains xbox. Find a way to separate them
                if "xbox" in device.name.lower():
                    print "Found xbox controller"
                    return 'xbox', device
                elif 'playstation' in device.name.lower():
                    print "Found ps3 controller"
                    return 'ps3', device
            except:
                pass

        xboxdrv_process.kill()
        exit("No controller found.")


if __name__ == '__main__':
    try:
        joystick_node = JoystickNode()
        joystick_node.start_node()
    except rospy.ROSInterruptException:
        pass
