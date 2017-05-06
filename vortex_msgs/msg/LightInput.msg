# Message for controlling lights on ROV
# List of strings contain the name of the light(s) to control,
# while the list of intensity contains the intensity of the corresponding light based on indexing in 
# the lists.
# For pwm-controlled light,intensity is in the range 0-100, while for bluetooth/raman intensity > 0 
# means on, 0 means off.
string[] toggle_light
uint8[] intensity
