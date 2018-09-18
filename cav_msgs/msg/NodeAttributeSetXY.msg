#
# NodeAttributeSetXY.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_NodeAttributeSetXY is a data frame used to convey one or more changes in the attribute set which occur
# at the node point at which it is used. Some of these attributes persist until the end of the lane or until changed again or
# turned off.
#
# NodeAttributeSetXY ::= SEQUENCE {
# localNode NodeAttributeXYList OPTIONAL,
# -- Attribute states which pertain to this node point
# disabled SegmentAttributeXYList OPTIONAL,
# -- Attribute states which are disabled at this node point
# enabled SegmentAttributeXYList OPTIONAL,
# -- Attribute states which are enabled at this node point
# -- and which remain enabled until disabled or the lane ends
# data LaneDataAttributeList OPTIONAL,
# -- Attributes which require an additional data values
# -- some of these are local to the node point, while others
# -- persist with the provided values until changed
# -- and this is indicated in each entry
# dWidth Offset-B10 OPTIONAL,
# -- A value added to the current lane width
# -- at this node and from this node onwards, in 1cm steps
# -- lane width between nodes are a linear taper between pts
# -- the value of zero shall not be sent here
# dElevation Offset-B10 OPTIONAL,
# -- A value added to the current Elevation
# -- at this node from this node onwards, in 10cm steps
# -- elevations between nodes are a linear taper between pts
# -- the value of zero shall not be sent here
# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-NodeAttributeSetXY}}OPTIONAL,
# ...
# }

# The NodeAttributeXYList data frame consists of a list of NodeAttributeXY entries.
# NodeAttributeXYList ::= SEQUENCE (SIZE(1..8)) OF NodeAttributeXY
j2735_msgs/NodeAttributeXY[] local_node
bool local_node_exists
# The DF_SegmentAttributeXYList data frame consists of a list of SegmentAttributeXY entries.
# SegmentAttributeXYList ::= SEQUENCE (SIZE(1..8)) OF SegmentAttributeXY
j2735_msgs/SegmentAttributeXY[] disabled
bool disabled_exists

j2735_msgs/SegmentAttributeXY[] enabled
bool enabled_exists

# The LaneDataAttributeList data frame consists of a list of LaneDataAttribute entries.
# LaneDataAttributeList ::= SEQUENCE (SIZE(1..8)) OF LaneDataAttribute
cav_msgs/LaneDataAttribute[] lane_attribute_list
bool data_exists

# Deviates from J2735 standard:
# dWidth in this message are conveyed as m rather than cm to avoid redundant unnecessary conversion by subscribers
# FLOAT (-5.12-5.11)
# -- a range of +- 5.11 meters
float32 dWitdh
bool dWitdh_exists

# Deviates from J2735 standard:
# dElevation in this message are conveyed as m rather than cm to avoid redundant unnecessary conversion by subscribers
float32 dElevation
bool dElevation_exists

# regional #TODO: RegionalExtensions are not yet implemented in asn1c