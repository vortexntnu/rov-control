# This message contains a Vortex ROV propulsion command

Header header

# Normalized motion command
# All elements must be in the range [-1, 1]
# The order is
# [forward, right, down, roll right, tilt up, turn right]
float64[6] motion

# Control mode
# Only one element may be set True
# The index of the True element determines mode,
# as defined in the ControlMode enum
bool[] control_mode
