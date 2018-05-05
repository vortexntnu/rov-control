#!/usr/bin/env python

# This is a nanopi neo 2+ / ROS port of:
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

STEPPER_PIN_VALUES = [[1, 0, 1, 0],
                      [0, 1, 1, 0],
                      [0, 1, 0, 1],
                      [1, 0, 0, 1]]


class Stepper:
    def __init__(self, number_of_steps, pins, pwm_pins, computer):
        """"Initialize 4-pin stepper motor.

        number_of_steps -- number of steps per revolution
        pins -- list of pin IDs for the stepper pins used (e.g. 'P9_01')
        pwm_pins -- pin IDs for stepper PWM pins used to enable or disable
        computer -- string identifier for computer type
        """
        self.computer = computer
        if self.computer == 'nanopi':
            import RPi.GPIO as GPIO
            print('stepper.py: Starting in nanopi mode')
        elif self.computer == 'pc-debug':
            print('stepper.py: Starting in PC debug mode, no stepper connected.')
        else:
            print('stepper.py: Invalid computer!')

        if self.computer != 'pc-debug':
            self.GPIO = GPIO

        self.curr_step = 0
        self.number_of_steps = number_of_steps

        self.pins = pins
        self.pwm_pins = pwm_pins
        if self.computer != 'pc-debug':
            self.GPIO.setmode(GPIO.BOARD)
            for pin in self.pins:
                self.GPIO.setup(pin, self.GPIO.OUT)
            for pwm_pin in self.pwm_pins:
                self.GPIO.setup(pwm_pin, self.GPIO.OUT)

    def step_once(self, direction):
        """"Step motor once in given direction."""
        if abs(direction) != 1:
            return
        self.curr_step += direction

        if self.computer != 'pc-debug':
            for i in range(len(self.pins)):
                pin_value = STEPPER_PIN_VALUES[self.curr_step % 4][i]
                if pin_value == 1:
                    self.GPIO.output(self.pins[i], self.GPIO.HIGH)
                elif pin_value == 0:
                    self.GPIO.output(self.pins[i], self.GPIO.LOW)
                else:
                    rospy.logwarn('Invalid output pin value.')
        else:
            print('Stepping: {0}'.format(
                STEPPER_PIN_VALUES[self.curr_step % 4]))

    def enable(self):
        if self.computer != 'pc-debug':
            for pin in self.pwm_pins:
                self.GPIO.output(pin, self.GPIO.HIGH)
        else:
            print('Enabling stepper')

    def disable(self):
        if self.computer != 'pc-debug':
            for pin in self.pwm_pins:
                self.GPIO.output(pin, self.GPIO.LOW)
        else:
            print('Disabling stepper')

    def shutdown(self):
        if self.computer == 'nanopi':
            print('stepper.py: Shutting down and cleaning GPIO pins.')
            self.GPIO.cleanup()
