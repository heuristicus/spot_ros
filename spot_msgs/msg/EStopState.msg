# Type
uint8 TYPE_UNKNOWN = 0
uint8 TYPE_HARDWARE = 1
uint8 TYPE_SOFTWARE = 2

# State
uint8 STATE_UNKNOWN = 0
uint8 STATE_ESTOPPED = 1
uint8 STATE_NOT_ESTOPPED = 2

Header header
string name
uint8 type
uint8 state
string state_description
