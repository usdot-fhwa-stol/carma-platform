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
# Units: mph
uint8 lower_speed_limit

# The upper speed limit
# Units: mph
uint8 speed_limit

# The lane markings found between interior lanes
cav_msgs/LaneEdgeType interior_lane_markings

# The lane markings on the left edge of the left most lane
cav_msgs/LaneEdgeType left_most_lane_marking

# The lane markings on the right edge of the right most lane
cav_msgs/LaneEdgeType right_most_lane_marking

# Marks the maximum negative cross track distance a vehicle can have and still be considered on a route
# Units: Meters
float64 min_cross_track

# Marks the maximum positive cross track distance a vehicle can have and still be considered on a route
# Units: Meters
float64 max_cross_track

# The index of the lane that this waypoint is located in
# Lanes are 0 indexed from right to left
uint8 lane_index

# The width of an individual lane in m
float32 lane_width

# A bit flag describing which of the following fields are set. 1 = set, 0 = not set
uint16 set_fields

# Bit flags which can be set in the set_fields
uint16 DISABLED_GUIDANCE_ALGORITHMS=1
uint16 LANE_CLOSURES=2
uint16 NEAREST_MILE_MARKER=4
uint16 NEEDED_MANEUVERS=8
uint16 REQUIRED_LANE_INDEX=16
uint16 ROAD_TYPE=32

# Each variable below corresponds to the bit in set_fields moving from MSB->LSM as you read down the list
# Example disabled_guidance_algorithms is the MSB (0x8000) and lane_closures is (0x4000)

# A list of guidance algorithms which are disabled on the segment which ends in this waypoint
string[] disabled_guidance_algorithms

# A list of lane closures described by index
uint8[] lane_closures

# The value of the nearest mile marker
float32 nearest_mile_marker

# A list of needed maneuvers
cav_msgs/Maneuver[] needed_maneuvers

# The index of the lane that the vehicle must enter to continue on its route
uint8 required_lane_index

# The roadway type which the waypoint applies to
cav_msgs/RoadType road_type
