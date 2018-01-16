#
# RoadwayObstacle.msg
# 
# Describes the location and geometry of an obstacle or vehicle in the roadway
# Obstacles are defined relative to the route with downtrack and crosstrack distances, and lane numbers
# The obstacle's size and velocity are described in a frame located on it's current route segment
#
# @author Michael McConnell
# @version 0.1
#

# The geometry of the vehicle
cav_msgs/ExternalObject object

# Enum describing the type of communication this vehicle is capable of
cav_msgs/ConnectedVehicleType connected_vehicle_type

# Cross-track distance of obstacle along route
float64 cross_track

# Down-track distance of obstacle along route
float64 down_track

# The lane this obstacle occupies
# Lanes are numbered according to the route file standards
# To be in a lane the obstacle's center of bounds must be in that lane
uint8 primary_lane

# A list of secondary lanes which this obstacle's bounding box intersects
uint8[] secondary_lanes

# The id of the waypoint this obstacle is approaching
# The route segment idex will by waypoint_id - 1
uint32 waypoint_id
