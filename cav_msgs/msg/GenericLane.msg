#
# GenericLane.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# GenericLane ::= SEQUENCE {
#
# laneID LaneID ::= INTEGER (0..255)
# -- The unique ID number assigned
# -- to this lane object
uint8 lane_id

# name DescriptiveName OPTIONAL := IA5String (SIZE(1..63))
# -- often for debug use only
# -- but at times used to name ped crossings
string name
bool name_exists

# ingressApproach ApproachID OPTIONAL, -- inbound
# ApproachID ::= INTEGER (0..15)
uint8 ingress_approach
bool ingress_approach_exists

# egressApproach ApproachID OPTIONAL, -- outbound
# -- Approach IDs to which this lane belongs
uint8 egress_approach
bool egress_approach_exists

# laneAttributes LaneAttributes,
# -- All Attribute information about
# -- the basic selected lane type
# -- Directions of use, Geometric co-sharing
# -- and Type Specific Attributes
# -- These Attributes are 'lane - global' that is,
# -- they are true for the entire length of the lane
j2735_msgs/LaneAttributes lane_attributes

# maneuvers AllowedManeuvers OPTIONAL,
# -- the permitted maneuvers for this lane
j2735_msgs/AllowedManeuvers maneuvers
bool maneuvers_exists

# nodeList NodeListXY,
# -- Lane spatial path information as well as
# -- various Attribute information along the node path
# -- Attributes found here are more general and may
# -- come and go over the length of the lane.
cav_msgs/NodeListXY node_list

# connectsTo ConnectsToList OPTIONAL,
# -- a list of other lanes and their signal group IDs
# -- each connecting lane and its signal group ID
# -- is given, therefore this element provides the
# -- information formerly in "signalGroups" in prior
# -- editions.
# The ConnectsToList data structure is used in the generic lane descriptions to provide a sequence of other defined
# lanes to which each lane connects beyond its stop point. See the Connection data frame entry for details. Note that this
# data frame is not used in some lane object types.
# ConnectsToList ::= SEQUENCE (SIZE(1..16)) OF Connection
j2735_msgs/Connection[] connect_to_list
bool connects_to_exists

# overlays OverlayLaneList OPTIONAL,
# -- A list of any lanes which have spatial paths that
# -- overlay (run on top of, and not simply cross)
# -- the path of this lane when used
# The Overlay Lane List data frame is a sequence of lane IDs which refers to lane objects that overlap or overlay the
# current lane's spatial path.
#  OverlayLaneList ::= SEQUENCE (SIZE(1..5)) OF LaneID
# -- The unique ID numbers for any lane object which have
# -- spatial paths that overlay (run on top of, and not
# -- simply cross with) the current lane.
# -- Such as a train path that overlays a motor vehicle
# -- lane object for a roadway segment.

# LaneID ::= INTEGER (0..255)
# -- the value 0 shall be used when the lane ID is
# -- not available or not known
# -- the value 255 is reserved for future use
uint16[] overlay_lane_list
bool overlay_lane_list_exists

# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-GenericLane} OPTIONAL,
# ...
# }