#
# RoadSegment.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_ RoadSegment data frame is a complete description of a RoadSegment including its geometry and its
# allowed navigational paths (independent of any additional regulatory restrictions that may apply over time or from user
# classification) and any current disruptions such as a work zone or incident event.
#
# RoadSegment ::= SEQUENCE {
# name DescriptiveName OPTIONAL,
# id RoadSegmentReferenceID,
# -- a globally unique value for the segment
# revision MsgCount,
# -- Required default values about the descriptions to follow
# refPoint Position3D,
# -- the reference from which subsequent
# -- data points are offset until a new
# -- point is used.
# laneWidth LaneWidth OPTIONAL,
# -- Reference width used by all subsequent
# -- lanes unless a new width is given
# speedLimits SpeedLimitList OPTIONAL,
# -- Reference regulatory speed limits
# -- used by all subsequent
# -- lanes unless a new speed is given
# -- See Section 11 for converting and
# -- translating speed expressed in mph
# -- into units of m/s
# -- Data describing disruptions in the RoadSegment
# -- such as work zones etc will be added here;
# -- in the US the SAE ITIS codes would be used here
# -- The details regarding each lane type in the RoadSegment
# roadLaneSet RoadLaneSetList,
# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-RoadSegment}} OPTIONAL,
# ...
# }

# name DescriptiveName OPTIONAL := IA5String (SIZE(1..63))
# -- often for debug use only
# -- but at times used to name ped crossings
string name
bool name_exists

j2735_msgs/RoadSegmentReferenceID id

#MsgCount ::= INTEGER (0..127)
uint8 revision

cav_msgs/Position3D ref_point

# Deviates from J2735 standard:
# dWidth in this message are conveyed as m rather than cm to avoid redundant unnecessary conversion by subscribers
# LaneWidth OPTIONAL ::= FLOAT (0..327.67) -- units of m
float32 lane_width
bool lane_width_exists

# SpeedLimitList ::= SEQUENCE (SIZE(1..9)) OF RegulatorySpeedLimit
cav_msgs/RegulatorySpeedLimit[] speed_limits
bool speed_limits_exists

# The DF_RoadLaneSetList data frame consists of a list of GenericLane entries used to describe a segment of
#  roadway.
#
# RoadLaneSetList ::= SEQUENCE (SIZE(1..255)) OF GenericLane
cav_msgs/GenericLane[] road_lane_set_list

#regional #TODO: RegionalExtensions are not yet implemented in asn1c




