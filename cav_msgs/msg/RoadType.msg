#
# RoadType.msg
# 
# An enum which describes different road types
#
# @author Michael McConnell
# @version 0.1
#

# Variable which can be used to store a specific road type enum value.
uint8 type

# A highway or freeway
uint8 FREEWAY=0

# An on or off ramp
uint8 RAMP=1

# A highcapacity urban or sub-urban road
uint8 ARTERIAL=2

# A secondary road [not a highway]
uint8 SECONDARY=3

# A parking lot
uint8 PARKING_LOT=4