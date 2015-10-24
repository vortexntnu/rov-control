
from std_msgs.msg import String
from controller.msg import *

def ps3_code_map(code):
    return {
            0:'X1',
            1:'Y1',

            3:'X2',
            4:'Y2',

            9:'STrigth',
            10:'STleft',

            16:'DX',
            17:'DY',

            310:'HTleft',
            311:'HTrigth',

            304:'A',
            305:'B',
            307:'X',
            308:'Y',

            314:'select',
            315:'start',
            316:'xbox',

            # Stick pushdown
            317:'LS',
            318:'RS'

            }.get(code,None)


def deadzone(value):
    if abs(value) < 3200:
        return 0
    return value


def read_events(device, model):
    events = device.read()

    for event in events:
        input = xbox_code_map(event.code)
        if input == 'X1':
            if event.value != 0:
                model.control_model['X1'] = event.value
        elif input == 'DY':
            if event.value == -1:
                model.control_model['north'] = 1
                model.control_model['south'] = 0
            elif event.value == 1:
                model.control_model['north'] = 0
                model.control_model['south'] = 1
            else:
                model.control_model['north'] = 0
                model.control_model['south'] = 0
        elif input == 'DX':
            if event.value == -1:
                model.control_model['west'] = 1
                model.control_model['east'] = 0
            elif event.value == 1:
                model.control_model['west'] = 0
                model.control_model['east'] = 1
            else:
                model.control_model['west'] = 0
                model.control_model['east'] = 0
        else: 
            model.control_model[input] = event.value
