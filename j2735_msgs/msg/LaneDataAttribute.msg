

# The data frame DF_LaneDataAttribute is used to relate an attribute and a control value at a node point or along a
# lane segment from an enumerated list of defined choices. It is then followed by a defined data value associated with it and
# which is defined elsewhere in this standard.


# LaneDataAttribute ::= CHOICE {
# -- Segment attribute types and the data needed for each
# pathEndPointAngle DeltaAngle,
# -- adjusts final point/width slant
# -- of the lane to align with the stop line
# laneCrownPointCenter RoadwayCrownAngle,
# -- sets the canter of the road bed
# -- from centerline point
# laneCrownPointLeft RoadwayCrownAngle,
# -- sets the canter of the road bed
# -- from left edge
# laneCrownPointRight RoadwayCrownAngle,
# -- sets the canter of the road bed
# -- from right edge
# laneAngle MergeDivergeNodeAngle,
# -- the angle or direction of another lane
# -- this is required to support Japan style
# -- when a merge point angle is required
# speedLimits SpeedLimitList,
# -- Reference regulatory speed limits
# -- used by all segments
# -- Add others as needed, in regional space
# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-LaneDataAttribute}},
# ...
# }

uint8 choice
uint8 PATH_END_POINT_ANGLE=0
uint8 LANE_CROWN_POINT_CENTER=1
uint8 LANE_CROWN_POINT_LEFT=2
uint8 LANE_CROWN_POINT_RIGHT=3
uint8 LANE_ANGLE=4
uint8 SPEED_LIMITS=5

# DeltaAngle ::= INTEGER (-150..150)
# -- With an angle range from
# -- negative 150 to positive 150
# -- in one degree steps where zero is directly
# -- along the axis or the lane center line as defined by the
# -- two closest points
#
# The DeltaAngle data element provides the final angle used in the last point of the lane path.
# Used to "cant" the stop line of the lane.

int16 path_end_point_angle

# RoadwayCrownAngle ::= INTEGER (-128..127)
# -- In LSB units of 0.3 degrees of angle
# -- over a range of -38.1 to + 38.1 degrees
# -- The value -128 shall be used for unknown
# -- The value zero shall be used for angles
# -- which are between -0.15 and +0.15
#
# The RoadwayCrownAngle data element relates the gross tangential angle of the roadway surface with respect to
# the local horizontal axis and is measured at the indicated part of the lane.

int8 lane_crown_point_center
int8 lane_crown_point_right
int8 lane_crown_point_left

# MergeDivergeNodeAngle ::= INTEGER (-180..180)
# -- In units of 1.5 degrees from north
# -- the value -180 shall be used to represent
# -- data is not available or unknown
int16 lane_angle

# SpeedLimitList ::= SEQUENCE (SIZE(1..9)) OF RegulatorySpeedLimit
j2735_msgs/SpeedLimitList speed_limits

# regional #TODO: RegionalExtensions are not yet implemented in asn1c
