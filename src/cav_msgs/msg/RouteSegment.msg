#
# RouteSegment.msg
# 
# A route segment. If on the route/current_segment topic it is the active segment. 
#
# @author Michael McConnell
# @version 0.1
#

# The downtrack waypoint of this segment [upcoming]
cav_msgs/RouteWaypoint waypoint

# The uptrack waypoint of this segment [previous]
cav_msgs/RouteWaypoint prevWaypoint

# The length of this segment
float64 length
