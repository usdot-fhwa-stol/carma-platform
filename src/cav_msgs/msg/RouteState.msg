#
# Route State Message
# 
# State of route progress including orthogonal distance from 
# vehicle to current route segment line, parametric distance along
# route, ETA, completion status, and the length of the route 
# provided in various metrics
#
# @author Michael McConnell
# @version 0.1
#

std_msgs/Header header
# The route id for which this message applies
uint32 routeID
# Indication of whether the route is completed
bool completed
# Cross-track distance of vehicle along route
float64 cross_track
# Down-track distance of vehicle along route
float64 down_track
# Expected time of arrival for vehicle at the end of route
time expectedTimeOfArrival
# A key value pairing of route lengths with different distance metrics
cav_msgs/KeyValue[] lengthMap 
