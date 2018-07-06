#
# NodeXY26b.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# A 26-bit node type with offset values from the last point in X and Y.

# Node-XY-26b ::= SEQUENCE {
# x Offset-B13,
# y Offset-B13
# }

# Offset-B13 ::= INTEGER (-4096..4095)
# -- a range of +- 40.95 meters
# A 13-bit delta offset in X or Y direction from some known point. For non-vehicle centric coordinate frames of
# reference, offset is positive to the East (X) and to the North (Y) directions. The most negative value shall be used to
# indicate an unknown value.

float32 x
float32 y