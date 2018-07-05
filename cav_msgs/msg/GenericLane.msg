#
# GenericLane.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# GenericLane ::= SEQUENCE {

# laneID LaneID ::= INTEGER (0..255)
# -- The unique ID number assigned
# -- to this lane object
uint8 lane_id

# name DescriptiveName OPTIONAL := IA5String (SIZE(1..63))
# -- often for debug use only
# -- but at times used to name ped crossings
string name

# ingressApproach ApproachID OPTIONAL, -- inbound
# ApproachID ::= INTEGER (0..15)
uint8 ingress_approach

# egressApproach ApproachID OPTIONAL, -- outbound
# -- Approach IDs to which this lane belongs
uint8 egress_approach

# laneAttributes LaneAttributes,
# -- All Attribute information about
# -- the basic selected lane type
# -- Directions of use, Geometric co-sharing
# -- and Type Specific Attributes
# -- These Attributes are 'lane - global' that is,
# -- they are true for the entire length of the lane
cav_msgs/LaneAttributes lane_attributes

# maneuvers AllowedManeuvers OPTIONAL,
# -- the permitted maneuvers for this lane
cav_msgs/AllowedManeuvers maneuvers

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

cav_msgs/ConnectsToList connects_to

# overlays OverlayLaneList OPTIONAL,
# -- A list of any lanes which have spatial paths that
# -- overlay (run on top of, and not simply cross)
# -- the path of this lane when used

cav_msgs/OverlayLaneList overlay_lane_list

# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-GenericLane} OPTIONAL,
# ...
# }