Header header

# Linear forward/backward motion command
# Positive forward
# Range [-1, 1]
float64 forward

# Linear right/left motion command
# Positive right
# Range [-1, 1]
float64 right

# Linear down/up motion command
# Positive down
# Range [-1, 1]
float64 down

# Angular tilt up/down command
# Positive up
# Range [-1, 1]
float64 tilt_up

# Angular turn right/left command
# Positive right
# Range [-1, 1]
float64 turn_right

# Control mode
# Set to 0 for open loop
# Set to 1 for position hold
# Set to 2 for depth hold
int32 control_mode
