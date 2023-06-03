# Use when requesting a change to the power state. Indicates that no action should be taken on that device. This is
# necessary because the BD API uses a bool to set the power state, where false is off. It would be impossible to know from
# a bool in a request to change power state whether the user wanted to turn the device off or leave it as it was.
uint8 NO_ACTION=0
# Indicates that the power is on, or requests the power to be turned on
uint8 POWER_ON=1
# Indicates that the power is off, or requests the power to be turned off
uint8 POWER_OFF=2

uint8 ptz
uint8 aux1
uint8 aux2
uint8 external_mic