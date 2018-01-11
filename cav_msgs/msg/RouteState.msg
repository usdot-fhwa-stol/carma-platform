#
# RouteState.msg
# 
# State of route progress including orthogonal distance from 
# vehicle to current route segment line, parametric distance along
# route, ETA, completion status, and the length of the route 
# provided in various metrics
#
# @author Michael McConnell
# @version 0.1
#

Header header

# The route id for which this message applies
string routeID

# Enum denoting the state of the route following
uint8 state

uint8 LOADING_ROUTES=0
uint8 ROUTE_SELECTION=1
uint8 WAITING_TO_START=2
uint8 FOLLOWING_ROUTE=3

# Enum denoting an event which has just occurred on the route
uint8 event

uint8 NONE=0
uint8 ROUTE_SELECTED=1
uint8 ROUTE_STARTED=2
uint8 ROUTE_COMPLETED=3
uint8 LEFT_ROUTE=4
uint8 ROUTE_ABORTED=5

# Cross-track distance of vehicle along route
float64 cross_track

# Down-track distance of vehicle along route
float64 down_track

# Down-track distance of vehicle along the current segment
float64 segment_down_track

# The route segment which the host vehicle is currently on
cav_msgs/RouteSegment current_segment
