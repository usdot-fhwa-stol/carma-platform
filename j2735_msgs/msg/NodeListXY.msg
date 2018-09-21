#
# NodeListXY.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The NodeListXY data structure provides the sequence of signed offset node point values for determining the Xs and
# Ys (and possibly Width or Zs when present), using the then current Position3D object to build a path for the centerline of
# the subject lane type. Each X,Y point is referred to as a Node Point. The straight line paths between these points are
# referred to as Segments.
#
# NodeListXY ::= CHOICE {
# nodes NodeSetXY,
# -- a lane made up of two or more
# -- XY node points and any attributes
# -- defined in those nodes
# computed ComputedLane,
# -- a lane path computed by translating
# -- the data defined by another lane
# ...
# }

uint8 choice
uint8 NODE_SET_XY=0
uint8 COMPUTED_LANE=1

j2735_msgs/NodeSetXY nodes
j2735_msgs/ComputedLane computed