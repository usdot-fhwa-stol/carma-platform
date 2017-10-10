#
# LaneEdgeType.msg
# 
# An enumeration which identifies the different types of edges in a lane.
#
# @author Michael McConnell
# @version 0.1
#

# Variable which can be used to store a specific lane edge type value.
uint8 type

# A solid yellow line which should not be crossed and has opposing traffic on the opposite side
uint8 SOLID_YELLOW=0

# A solid white line which should not be crossed
uint8 SOLID_WHITE=1

# A broken yellow line which can be crossed to pass but not as a lane merge
uint8 BROKEN_YELLOW=2

# A broken white line which can be crossed to pass or merge
uint8 BROKEN_WHITE=3

# An impassible marking on the road such as construction barriers
uint8 IMPASSIBLE=4
