# DriverStatus.msg
#
# Announces the status and category of software driver publishing this message.
# Some drivers may cover multiple categories because the hardware they
# represent has multiple capabilities.  
#

# driver_name is the ROS name of the driver node
string  name

# status indicates the operational state of the underlying hardware device,
# and is an enumerated value.
uint8  status
# enumeration values for status:
uint8  OFF=0
uint8  OPERATIONAL=1
uint8  DEGRADED=2
uint8  FAULT=3
#					<other values reserved for future expansion>

# These are the possible categories - at least one must be true
bool    can_bus
bool    sensor
bool    position
bool    comms
bool    controller
