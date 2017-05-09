#!/usr/bin/env python

# This is a BeagleBone / ROS port of:
# Stepper.cpp - Stepper library for Wiring/Arduino - Version 1.1.0
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.

import rospy

COMPUTER = rospy.get_param('/computer')
if COMPUTER == 'raspberry':
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    pass
elif COMPUTER == 'beaglebone':
    import Adafruit_BBIO.GPIO as GPIO
else:
    rospy.logfatal('Invalid COMPUTER, shutting down...')
    rospy.signal_shutdown('')

MICROSEC_PER_SEC = 1000 * 1000
STEPPER_PIN_VALUES = [[1, 0, 1, 0],
                      [0, 1, 1, 0],
                      [0, 1, 0, 1],
                      [1, 0, 0, 1]]


def curr_time_in_microsec():
    return rospy.Time.now().to_sec() * MICROSEC_PER_SEC


class Stepper():
    def __init__(self, number_of_steps, pins):
        """"Initialize 4-pin stepper motor.

        number_of_steps -- number of steps per revolution
        pins -- list of pin IDs for the stepper pins used (e.g. "P9_01")
        """
        self.curr_step = 0
        self.direction = 0
        self.last_step_time = curr_time_in_microsec()
        self.number_of_steps = number_of_steps

        self.pins = pins
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)

    def set_speed(self, speed):
        """"Set stepper speed in RPM."""
        self.step_delay = (60 * MICROSEC_PER_SEC / self.number_of_steps) / speed

    def step(self, steps_to_move):
        """Move motor specified number of steps. Negative number for reverse."""
        steps_left = abs(steps_to_move)

        self.direction = 1 if steps_to_move > 0 else 0

        while steps_left > 0:
            now = curr_time_in_microsec()
            if (now - self.last_step_time) >= self.step_delay:
                self.last_step_time = curr_time_in_microsec()
                if self.direction == 1:
                    self.curr_step += 1
                    if self.curr_step == self.number_of_steps:
                        self.curr_step = 0
                else:
                    if self.curr_step == 0:
                        self.curr_step = self.number_of_steps
                    self.curr_step -= 1

                steps_left -= 1

                self.step_once(self.curr_step % 4)

    def step_once(self, curr_step):
        """"Step motor once."""
        for i in range(len(self.pins)):
            pin_value = STEPPER_PIN_VALUES[curr_step][i]
            if pin_value == 1:
                GPIO.output(self.pins[i], GPIO.HIGH)
            elif pin_value == 0:
                GPIO.output(self.pins[i], GPIO.LOW)
            else:
                rospy.logwarn('Invalid output pin value.')
