# https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#streamparams
# White balance modes
int8 OFF=-1
# This is not provided, but we add it to be able to distinguish when setting the white balance
int8 UNSET=0
int8 AUTO=1
int8 INCANDESCENT=2
int8 FLUORESCENT=3
int8 WARM_FLUORESCENT=4
int8 DAYLIGHT=5
int8 CLOUDY=6
int8 TWILIGHT=7
int8 SHADE=8
int8 DARK=9

# Compression level target in bits per second
int64 target_bitrate
# How often should the entire feed be refreshed (in frames)
int64 refresh_interval
# How often should an IDR message be sent (in frames)
int64 idr_interval
# Automatic white balance
int8 awb