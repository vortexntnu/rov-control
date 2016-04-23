# ROV motion commands:

# Sideways, positive right
int32 strafe_X

# Forward/back, positive forward
int32 strafe_Y

# Turn, positive right
int32 turn_X

# Tilt, positive up
int32 turn_Y

# Up/down, positive up, -255 to 255
int32 ascend

# Control mode, 0 = open loop mode, 1 = stationkeeping mode
int32 control_mode
