#
# CommunicationClass.msg
# 
# An enumeration which identifies the different types of connected vehicle communication classes
#
# @author Michael McConnell
# @version 0.1
#

# A vehicle with no communications
uint8 UNCONNECTED=0

# A vehicle can send messages but not recieve them
uint8 ONEWAY_SEND=1

# A vehicle can recieve messages but not send them
uint8 ONEWAY_RECIEVE=2

# A vehicle can both send and recieve messages
uint8 TWOWAY=3


