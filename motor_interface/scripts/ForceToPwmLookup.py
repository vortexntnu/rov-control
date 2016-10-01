#!/usr/bin/env python
# -*- coding: utf-8 -*-


ForceLookup = [
    -5.7056872745,
    -5.2153547745,
    -4.6358709118,
    -4.1901140882,
    -3.92266,
    -3.6106302255,
    -3.2094490882,
    -2.6745409118,
    -2.2733597745,
    -1.96133,
    -1.6493002255,
    -1.3818461373,
    -1.0252406863,
    -0.7132109118,
    -0.4903325,
    -0.3120297745,
    -0.133727049,
    -0.0891513627,
    0,
    0,
    0,
    0,
    0.0891513627,
    0.356605451,
    0.624059549,
    1.0252406863,
    1.4264218137,
    1.9167543137,
    2.4962381863,
    3.0311463627,
    3.6106302255,
    4.2346897745,
    4.7250222745,
    5.3490818137,
    6.017717049,
    6.730927951,
    7.221260451,
    7.8898956863,
    8.6476822745,
    9.450044549
]

PulseWidthMin = 1300
PulseWidthMax = 1700
PulseWidthIncrement = 10


def ForceToMicroSec(force):
    us = PulseWidthMax
    for i in range(0, len(ForceLookup)):
        if force < ForceLookup[i]:

            us_prev = PulseWidthMin + PulseWidthIncrement*(i-1)
            us_next = PulseWidthMin + PulseWidthIncrement*i
            us_diff = us_next - us_prev
            force_diff = ForceLookup[i] - ForceLookup[i-1]
            force_offset = force - ForceLookup[i-1]
            us = us_prev + us_diff*(force_offset/force_diff)

            return us


# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

def MicroSecToPwmValue(us):
    pulse = float(us)
    pulse_length = 1000000.0    # 1,000,000 us per second
    pulse_length /= 273.0      # 273 Hz (målt frekvens)
    #print('{0}us per period'.format(pulse_length))
    pulse_length /= 4096     # 12 bits of resolution
    #print('{0}us per bit'.format(pulse_length))
    pulse /= pulse_length
    return pulse


def ForceToPwm(force):
    return MicroSecToPwmValue(ForceToMicroSec(force));

