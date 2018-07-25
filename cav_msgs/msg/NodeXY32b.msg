#
# NodeXY32b.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# A 32-bit node type with offset values from the last point in X and Y.

# Node-XY-32b ::= SEQUENCE {
# x Offset-B16,
# y Offset-B16
# }

# A 16-bit delta offset in X, Y or Z direction from some known point. For non-vehicle centric coordinate frames of
# reference, offset is positive to the East (X) and to the North (Y) directions. The most negative value shall be used to
# indicate an unknown value.
# Offset-B16 ::= INTEGER (-32768..32767)
# -- a range of +- 327.68 meters

float32 x
float32 y