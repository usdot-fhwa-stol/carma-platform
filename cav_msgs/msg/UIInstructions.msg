#
# UIInstructions.msg
# 
# Describes simple instructions for the CARMA UI to preform
#
# @version 0.1
#

# Time stamp
time stamp

# The message to send
string msg

# Enum identifying the contents of this message as informational or requireing acknowledgement
uint8 type

uint8 INFO=0 # Notification of status or state change
uint8 ACK_REQUIRED=1 # A command requiring driver acknowledgement
uint8 NO_ACK_REQUIRED=2 # A command that does not require driver acknowledgement

# Name of a simple service to be called when a acknowledgement is required (service must be std_msgs/Bool)
string response_service
