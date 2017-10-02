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

uint8 WAITING_TO_START=0
uint8 FOLLOWING_ROUTE=1

# Cross-track distance of vehicle along route
float64 cross_track

# Down-track distance of vehicle along route
float64 down_track
