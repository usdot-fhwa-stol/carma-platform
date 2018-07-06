#
# NodeXY20b.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# A 20-bit node type with offset values from the last point in X and Y.
#
# Node-XY-20b ::= SEQUENCE {
# x Offset-B10,
# y Offset-B10
# }

# Offset-B10 ::= INTEGER (-512..511)
# -- a range of +- 5.11 meters
# A 10-bit delta offset in X, Y or Z direction from some known point. For non-vehicle centric coordinate frames of
# reference, offset is positive to the East (X) and to the North (Y) directions. The most negative value shall be used to
# indicate an unknown value.

float32 x
float32 y