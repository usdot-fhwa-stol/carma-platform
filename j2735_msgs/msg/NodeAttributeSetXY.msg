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

j2735_msgs/NodeAttributeXYList local_node
bool local_node_exists

j2735_msgs/SegmentAttributeXYList disabled
bool disabled_exists

j2735_msgs/SegmentAttributeXYList enabled
bool enabled_exists

j2735_msgs/LaneDataAttributeList data
bool data_exists

# Offset-B10 ::= INTEGER (-512..511)
# -- a range of +- 5.11 meters
float32 dWitdh
bool dWitdh_exists

float32 dElevation
bool dElevation_exists

# regional #TODO: RegionalExtensions are not yet implemented in asn1c