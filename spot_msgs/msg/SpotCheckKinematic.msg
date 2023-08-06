# Errors reflect an issue with robot hardware.
uint8 ERROR_UNKNOWN = 0       # Unused enum.
uint8 ERROR_NONE = 1          # No hardware error detected.
uint8 ERROR_CLUTCH_SLIP = 2   # Error detected in clutch performance.
uint8 ERROR_INVALID_RANGE_OF_MOTION = 3  # Error if a joint has an incorrect range of motion.

# A flag to indicate if results has an error.
uint8 error

# The current offset [rad]
float32 offset
# The previous offset [rad]
float32 old_offset

# Joint calibration health score. range [0-1]
# 0 indicates an unhealthy kinematic joint calibration
# 1 indicates a perfect kinematic joint calibration
# Typically, values greater than 0.8 should be expected.
float32 health_score