uint8 ERROR_UNKNOWN = 0              # Unused enum.
uint8 ERROR_NONE = 1                 # No hardware error detected.
uint8 ERROR_ZERO_OUT_OF_RANGE = 2    # Load cell calibration failure.

# The loadcell error code
uint8 error
# The current loadcell zero as fraction of full range [0-1]
float32 zero
# The previous loadcell zero as fraction of full range [0-1]
float32 old_zero