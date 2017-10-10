#
# RouteWaypoint.msg
# 
# The waypoint of a route
#
# @author Michael McConnell
# @version 0.1
#

# The waypoint id number
uint32 waypoint_id

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
float64 altitude

# The number of lanes on the current road in the current direction of travel
# Must be set with value >= 1
uint8 lane_count

# The lower speed limit
uint8 lower_speed_limit

# The upper speed limit
uint8 speed_limit

# The lane markings found between interior lanes
cav_msgs/LaneEdgeType interior_lane_markings

# The lane markings on the left edge of the left most lane
cav_msgs/LaneEdgeType left_most_lane_marking

# The lane markings on the right edge of the right most lane
cav_msgs/LaneEdgeType right_most_lane_marking

# A bit flag describing which of the following fields are set. 1 = set, 0 = not set
uint16 set_fields

# A list of guidance algorithms which are disabled on the segment which ends in this waypoint
string[] disabled_guidance_algorithms

# A list of lane closures described by index
uint8[] lane_closures

# The value of the nearest mile marker
float32 nearest_mile_marker

# A list of needed maneuvers
# cav_msgs/Maneuver[] needed_maneuvers

# The index of the lane that the vehicle must enter to continue on its route
uint8 required_lane_index

# The roadway type which the waypoint applies to
cav_msgs/RoadType road_type
