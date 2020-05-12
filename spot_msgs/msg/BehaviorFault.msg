# Cause
uint8 CAUSE_UNKNOWN = 0
uint8 CAUSE_FALL = 1
uint8 CAUSE_HARDWARE = 2

# Status
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_CLEARABLE = 1
uint8 STATUS_UNCLEARABLE = 2

Header header
uint32 behavior_fault_id
uint8 cause
uint8 status
