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

# id IntersectionReferenceID,
# -- A globally unique value set,
# -- consisting of a regionID and
# -- intersection ID assignment
cav_msgs/IntersectionReferenceID id

# revision MsgCount  ::= INTEGER (0..127)
# -- Required default values about lane descriptions follow
uint8 revision

# refPoint Position3D, -- The reference from which subsequent
# -- data points are offset until a new
# -- point is used.
cav_msgs/Position3D ref_point

# laneWidth LaneWidth OPTIONAL ::= INTEGER (0..32767) -- units of 1 cm
# -- Reference width used by all subsequent
# -- lanes unless a new width is given
uint16 lane_width

# speedLimits SpeedLimitList OPTIONAL
# -- Reference regulatory speed limits
# -- used by all subsequent
# -- lanes unless a new speed is given
# -- See Section 11 for converting and
# -- translating speed expressed in mph
# -- into units of m/s
# -- Complete details regarding each lane type in this intersection
cav_msgs/SpeedLimitList speed_limits

# laneSet LaneList,
# -- Data about one or more lanes
# -- (all lane data is found here)
# -- Data describing how to use and request preemption and
# -- priority services from this intersection (if supported)
cav_msgs/LaneList lane_set

# -- NOTE Additonal data may be added in the next release of the
# -- standard at this point to handle this concept
# preemptPriorityData PreemptPriorityList OPTIONAL,
# -- data about one or more regional
# -- preempt or priority zones
cav_msgs/PreemptPriorityList preempt_priority_data


# TODO: Discuss how to implement RegionalExtension types.
# regional SEQUENCE (SIZE(1..4)) OF
# RegionalExtension {{REGION.Reg-IntersectionGeometry}} OPTIONAL,
# ...
# }
