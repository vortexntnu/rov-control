#!/usr/bin/env python3

import os, glob
import ctrlmodels

from evdev import InputDevice, ecodes
import asyncio
from asyncore import file_dispatcher, loop
from functools import partial

import rospy
from std_msgs.msg import String
from controller.msg import *


def init_device():
    devices = glob.glob('/dev/input/event*')
    for device in devices:
        print("trying device named " + device)
        try:
            device = InputDevice(device)
            print(device.name.lower())
            return('xbox', device)
            # if 'xbox' in device.name.lower():
            #     print("Found xbox controller in device " + device)
            #     return('xbox', device)
            # elif 'playstation' in device.name.lower():
            #     print("Found ps3 controller")
            #     return('ps3', device)
        except:
            print("Nope")
            pass
    exit("No controller found. Have you remembered running xboxdrv?")



@asyncio.coroutine
def reader(pub, model):
    while not rospy.is_shutdown():
        yield from asyncio.sleep(0.1)
        pub.publish(model.create_msg())


def dummy_ctrl():
    return control(
        strafe_X = 20000,
        strafe_Y = 20000,

        turn_X = 10000,
        turn_Y = 0,
        
        ascend=1,
        descend=0
    )

def test():
    rospy.init_node('TEST', anonymous=True)
    pub = rospy.Publisher('Control', control, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(test())
        print("I am talking")
        rate.sleep()


def start_node():
    device = init_device()

    if(device[0] == 'xbox'):
        import xctrl
        model = xctrl.xboc_ctrl()
        read_events = xctrl.read_events

    if(device[0] == 'ps3'):
        import xctrl
        model = xctrl.xboc_ctrl()
        read_events = xctrl.read_events

    rospy.init_node( (device[0] + "controller") )
    pub = rospy.Publisher('Control', control, queue_size=10)

    loop = asyncio.get_event_loop()
    loop.add_reader(device[1], partial(read_events, device[1], model))
    loop.run_until_complete(reader(pub, model))



if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass




