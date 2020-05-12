# Severity
uint8 SEVERITY_UNKNOWN = 0
uint8 SEVERITY_INFO = 1
uint8 SEVERITY_WARN = 2
uint8 SEVERITY_CRITICAL = 3

Header header
string name
duration duration
int32 code
uint64 uid
string error_message
string[] attributes
uint8 severity
