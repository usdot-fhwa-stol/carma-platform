#
# Route Message
# 
# The segments of an active route
#
# @author Michael McConnell
# @version 0.1
#

Header header

# The route id
string routeID

# A list of segments which comprise the route
cav_msgs/RouteSegment[] segments
