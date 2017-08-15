#
# RouteUpdateNotification.msg
# 
# Provides route update information for routes which are changed while being followed by a vehicle. 
#
# @author Michael McConnell
# @version 0.1
#

std_msgs/Header header

# The route id for which this message applies
string route_ID

# The distance downtrack to the first modified segment
float64 distance_to_event

# The index of the first modified route segment in the active route
uint32 first_modification_index

# A list of all modified segments
cav_msgs/RouteSegment[] modified_segments

