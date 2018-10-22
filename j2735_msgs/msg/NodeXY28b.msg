#
# NodeXY28b.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# A 28-bit node type with offset values from the last point in X and Y.

# Node-XY-28b ::= SEQUENCE {
# x Offset-B14,
# y Offset-B14
# }

# A 14-bit delta offset in X or Y direction from some known point. For non-vehicle centric coordinate frames of
# reference, offset is positive to the East (X) and to the North (Y) directions.
# Offset-B14 ::= INTEGER (-8192..8191)
# -- a range of +- 81.91 meters

float32 x
float32 y
