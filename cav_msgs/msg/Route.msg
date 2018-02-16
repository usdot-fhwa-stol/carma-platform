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

# The route name for display to users
string routeName

# The validation status of this route
# True if this route can be safely used in the CARMA platform
bool valid

# A list of segments which comprise the route
cav_msgs/RouteSegment[] segments
