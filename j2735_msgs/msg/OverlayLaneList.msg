#
# OverlayLaneList.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The Overlay Lane List data frame is a sequence of lane IDs which refers to lane objects that overlap or overlay the
# current lane's spatial path.
#  OverlayLaneList ::= SEQUENCE (SIZE(1..5)) OF LaneID
# -- The unique ID numbers for any lane object which have
# -- spatial paths that overlay (run on top of, and not
# -- simply cross with) the current lane.
# -- Such as a train path that overlays a motor vehicle
# -- lane object for a roadway segment.

# LaneID ::= INTEGER (0..255)
# -- the value 0 shall be used when the lane ID is
# -- not available or not known
# -- the value 255 is reserved for future use
uint16[] overlay_lane_list