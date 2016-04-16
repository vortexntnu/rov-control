

def read_ps3_into_control_msg(device, model):

    for event in device.read_loop():
        input = ps3_code_map(event.code)
        if input == 'X1':
            if event.value != 0:
                model.strafe_X = deadzone(event.value)
        elif input == 'Y1':
            if event.value != 0:
                model.strafe_Y = deadzone(event.value)
        elif input == 'X2':
            if event.value != 0:
                model.turn_X = deadzone(event.value)
        elif input == 'Y2':
            if event.value != 0:
                model.turn_Y = deadzone(event.value)

        elif input == 'R1':
            model.lights_increase = (event.value == 1)
        elif input == 'L1':
            model.lights_decrease = (event.value == 1)

        elif input == 'TRIANGLE':
            model.lights_on = (event.value == 1)
        elif input == 'SQUARE':
            model.lights_off = (event.value == 1)
        elif input == 'CIRCLE':
            model.arm_grip_open = (event.value == 1)
        elif input == 'X':
            model.arm_grip_close = (event.value == 1)

        elif input == 'DY':
            model.arm_base = event.value
        elif input == 'DX':
            model.arm_rot = event.value

        elif input == 'L2':
            model.ascend = event.value
        elif input == 'R2':
            model.descend = event.value

        elif input == 'START':
            model.free_roam = (event.value == 1)
        elif input == 'SELECT':
            model.hold_position = (event.value == 1)

def read_xbox_into_control_msg(device, model):

    for event in device.read_loop():
        input = ps3_code_map(event.code)
        if input == 'X1':
            model.strafe_X = event.value
        elif input == 'Y1':
            model.strafe_Y = event.value
        elif input == 'X2':
            model.turn_Y = event.value
        elif input == 'Y2':
            model.turn_Y = event.value

        elif input == 'HTrigth':
            model.lights_increase = event.value
        elif input == 'HTleft':
            model.lights_decrease = event.value

        elif input == 'Y':
            model.lights_on = event.value
        elif input == 'X':
            model.lights_off = event.value
        elif input == 'A':
            model.arm_grip_open = not (event.value == 0)
        elif input == 'B':
            model.arm_grip_close = not (event.value == 0)

        elif input == 'DY':
            model.arm_base = event.value
        elif input == 'DX':
            model.arm_rot = event.value

        elif input == 'STright':
            model.ascend = event.value
        elif input == 'STleft':
            model.descend = event.value


def deadzone(value):
    if abs(value) < 3200:
        return 0
    return value


def ps3_code_map(code):
    return {
            0:'X1',
            1:'Y1',

            3:'X2',
            4:'Y2',

            9:'R2',
            10:'L2',

            16:'DX',
            17:'DY',

            310:'L1',
            311:'R1',

            304:'X',
            305:'CIRCLE',
            307:'SQUARE',
            308:'TRIANGLE',

            314:'SELECT',
            315:'START',
            316:'PS3',

            317:'L3',
            318:'R3'

            }.get(code,None)


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

