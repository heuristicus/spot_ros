# Errors reflect an issue with payload configuration.

# Unused enum.
uint8 ERROR_UNKNOWN = 0
# No error found in the payloads.
uint8 ERROR_NONE = 1
# There is a mass discrepancy between the registered payload and what is estimated.
uint8 ERROR_MASS_DISCREPANCY = 2

# A flag to indicate if configuration has an error.
uint8 error

# Indicates how much extra payload (in kg) we think the robot has
# Positive indicates robot has more payload than it is configured.
# Negative indicates robot has less payload than it is configured.
float32 extra_payload