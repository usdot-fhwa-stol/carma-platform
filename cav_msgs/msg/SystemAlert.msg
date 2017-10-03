# SystemAlert.msg
# 
# Broadcasts important alerts related to system health.
#

# the enumerated alert type
uint8   type

# enumeration values for type. 
# NOT_READY indicates that system is starting up
# DRIVERS_READY is a one-time indicator that the startup sequence is complete.
uint8   CAUTION = 1
uint8   WARNING = 2
uint8   FATAL = 3
uint8   NOT_READY = 4
uint8   DRIVERS_READY = 5
uint8   SHUTDOWN = 6

# the specific alert message in human readable form
string  description
