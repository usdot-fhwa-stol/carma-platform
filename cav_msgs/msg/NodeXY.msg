#
# NodeXY.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_NodeXY data frame presents a structure to hold data for a single node point in a path. Each selected node
# has an X and Y offset from the prior node point (or a complete lat-long representation in some cases) as well as optional
# attribute information.
#
# NodeXY ::= SEQUENCE {
# delta NodeOffsetPointXY,
# -- A choice of which X,Y offset value to use
# -- this includes various delta values as well a regional choices
# attributes NodeAttributeSetXY OPTIONAL,
# -- Any optional Attributes which are needed
# -- This includes changes to the current lane width and elevation
# ...
# }

cav_msgs/NodeOffsetPointXY delta

cav_msgs/NodeAttributeSetXY attributes
bool attributes_exists
