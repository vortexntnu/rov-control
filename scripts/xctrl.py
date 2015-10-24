
from std_msgs.msg import String
from controller.msg import *

def xbox_code_map(code):
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
        # print(event.code)
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


class xboc_ctrl():
    def __init__(self):
        self.control_model = {
            #LT
            'X1' : 0,
            'Y1' : 0,

            #RT
            'X2' : 0,
            'Y2' : 0,

            #Dpad
            'north' : 0,
            'west' : 0,
            'east' : 0,
            'south' : 0,

            'STleft' : 0,
            'STrigth' : 0,

            'HTleft' : 0.0,
            'HTrigth' : 0.0,

            'A' : 0,
            'B' : 0,
            'X' : 0,
            'Y' : 0,

            'start' : 0,
            'back' : 0,
        }

    def create_msg(self):
        return control(
            strafe_X = deadzone(self.control_model['X1']),
            strafe_Y = deadzone(self.control_model['Y1']),

            turn_X = deadzone(self.control_model['X2']),
            turn_Y = deadzone(self.control_model['Y2']),
            
            ascend=self.control_model['A'],
            descend=self.control_model['B']
        )
