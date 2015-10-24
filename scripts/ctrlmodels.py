




class ps3_ctrl():

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
