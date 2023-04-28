# https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#getbitstatusresponse-degradation
# Different degradation types
uint8 TYPE_STORAGE=0
uint8 TYPE_PTZ=1
uint8 TYPE_LED=2

# The system affected by the degradation
uint8 type
# What degradation is being experienced
string description