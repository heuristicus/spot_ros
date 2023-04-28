# This message covers two different types which have the exact same field names, to reduce duplication. Depending on the topic,
# The meaning of the pan/tilt/zoom values changes.
# https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#ptzposition
# https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#ptzvelocity
std_msgs/Header header
# Description of the ptz
spot_cam/PTZDescription ptz
# degrees or degrees per second
float32 pan
# degrees or degrees per second
float32 tilt
# zoom level or zoom level per second
float32 zoom