# Errors reflect an issue with hip range of motion
uint8 ERROR_UNKNOWN = 0
uint8 ERROR_NONE = 1
uint8 ERROR_OBSTRUCTED = 2

uint8 error

# The measured angles (radians) of the HX and HY joints where the obstruction was detected
float32[] hx
float32[] hy