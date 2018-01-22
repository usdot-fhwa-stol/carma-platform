#
# ConnectedVehicleType.msg
# 
# An enumeration which identifies the different types of connected vehicle communication types
#
# @author Michael McConnell
# @version 0.1
#

# Variable which can be used to store a specific communication class value.
uint8 type

uint8 NOT_CONNECTED=0 # Vehicle has no known communication capabilities
uint8 CONNECTED=1 # Vehicle is known to be connected as BSMs are being received. No other assumptions are made about its capabilities.
uint8 CONNECTED_AND_AUTOMATED=2 # CAV: Vehicle is known to be connected and automated


