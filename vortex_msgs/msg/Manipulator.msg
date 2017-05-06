# Manipulator servo/stepper commands

# Normalized claw position
# Must be in the range [-1, 1]
float64 claw_position

# Valve spinning
# Set either true for corresponding action
# Do not set both true
bool open_valve
bool close_valve
