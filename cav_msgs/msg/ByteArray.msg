#
# ByteArray.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# Raw binary blobs from Drivers.Comms package and then parse the raw data according to the header. 
# Currently all messages are DSRC message. 
# 
# NOTE: Initial version only considers Part I. Part II will be defined for later use. 

std_msgs/Header header

# Need to know the type before parsing. MessageTypes are MAP, SPAT, TIM, BSM, Mobility.
string messageType

# The DSRC message content
uint8[] content
