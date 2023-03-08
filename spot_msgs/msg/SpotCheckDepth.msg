uint8 STATUS_UNKNOWN = 0     # Unused enum.
uint8 STATUS_OK = 1          # No detected calibration error.
uint8 STATUS_WARNING = 2     # Possible calibration error detected.
uint8 STATUS_ERROR = 3       # Error with robot calibration.

uint8 status
float32 severity_score