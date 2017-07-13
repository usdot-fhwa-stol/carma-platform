#
# Current Segment Message
# 
# The active segment of the current route
#
# @author Michael McConnell
# @version 0.1
#

std_msgs/Header header

# The lead waypoint of this segment
cav_msgs/route_waypoint waypoint

# The previous waypoint of this segment
cav_msgs/route_waypoint prevWaypoint
