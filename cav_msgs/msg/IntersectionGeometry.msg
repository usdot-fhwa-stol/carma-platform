#
# IntersectionGeometry.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# A complete description of an intersection's roadway geometry and its allowed navigational paths
# (independent of any additional regulatory restrictions that may apply over time or from user classification).

# IntersectionGeometry ::= SEQUENCE {
# name DescriptiveName OPTIONAL ::= IA5String (SIZE(1..63))
# -- For debug use only
string name
bool name_exists

# id IntersectionReferenceID,
# -- A globally unique value set,
# -- consisting of a regionID and
# -- intersection ID assignment
j2735_msgs/IntersectionReferenceID id

# revision MsgCount  ::= INTEGER (0..127)
# -- Required default values about lane descriptions follow
uint8 revision

# refPoint Position3D, -- The reference from which subsequent
# -- data points are offset until a new
# -- point is used.
cav_msgs/Position3D ref_point

# Deviates from J2735 standard:
# LaneWidth in this message are conveyed as m rather than cm to avoid redundant unnecessary conversion by subscribers
# laneWidth LaneWidth OPTIONAL ::= FLOAT32 (0..327.67) -- units of m
# -- Reference width used by all subsequent
# -- lanes unless a new width is given
float32 lane_width
bool lane_width_exists

# speedLimits SpeedLimitList OPTIONAL
# -- Reference regulatory speed limits
# -- used by all subsequent
# -- lanes unless a new speed is given
# -- See Section 11 for converting and
# -- translating speed expressed in mph
# -- into units of m/s
# -- Complete details regarding each lane type in this intersection
# SpeedLimitList ::= SEQUENCE (SIZE(1..9)) OF RegulatorySpeedLimit
cav_msgs/RegulatorySpeedLimit[] speed_limits
bool speed_limits_exists

# laneSet LaneList,
# -- Data about one or more lanes
# -- (all lane data is found here)
# -- Data describing how to use and request preemption and
# -- priority services from this intersection (if supported)
# The LaneList data frame consists of a list of GenericLane entries.
# LaneList ::= SEQUENCE (SIZE(1..255)) OF GenericLane
cav_msgs/GenericLane[] lane_list


# -- NOTE Additonal data may be added in the next release of the
# -- standard at this point to handle this concept
# preemptPriorityData PreemptPriorityList OPTIONAL,
# -- data about one or more regional
# -- preempt or priority zones
# The DF_PreemptPriorityList data frame consists of a list of RegionalSignalControlZone entries.
#
# PreemptPriorityList ::= SEQUENCE (SIZE(1..32)) OF SignalControlZone
j2735_msgs/SignalControlZone[] preempt_priority_list
bool preempt_priority_data_exists

#TODO: RegionalExtensions are not yet implemented in asn1c
# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-IntersectionGeometry}} OPTIONAL,
# ...
# }
