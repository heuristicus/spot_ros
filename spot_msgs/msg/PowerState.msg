# MotorPowerState
uint8 STATE_UNKNOWN = 0
uint8 STATE_OFF = 1
uint8 STATE_ON = 2
uint8 STATE_POWERING_ON = 3
uint8 STATE_POWERING_OFF = 4
uint8 STATE_ERROR = 5

# ShorePowerState
uint8 STATE_UNKNOWN_SHORE_POWER = 0
uint8 STATE_ON_SHORE_POWER = 1
uint8 STATE_OFF_SHORE_POWER = 2

Header header
uint8 motor_power_state
uint8 shore_power_state
float64 locomotion_charge_percentage
duration locomotion_estimated_runtime
