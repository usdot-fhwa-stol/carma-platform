#
# NodeXY22b.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# A 22-bit node type with offset values from the last point in X and Y.

# Node-XY-22b ::= SEQUENCE {
# x Offset-B11,
# y Offset-B11
# }

# Offset-B11 ::= INTEGER (-1024..1023)
# -- a range of +- 10.23 meters
# An 11-bit delta offset in X or Y direction from some known point. For non-vehicle centric coordinate frames of
# reference, offset is positive to the East (X) and to the North (Y) directions. The most negative value shall be used to
# indicate an unknown value.

float32 x
float32 y