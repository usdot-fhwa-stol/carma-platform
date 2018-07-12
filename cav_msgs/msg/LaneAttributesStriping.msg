#
# LaneAttributesBarrier.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The LaneAttributes-Striping data element relates specific properties found
# in various types of ground striping lane types.
#
# LaneAttributes-Striping ::= BIT STRING {
# -- With bits as defined:
# stripeToConnectingLanesRevocableLane(0),
# -- this lane may be activated or not activated based
# -- on the current SPAT message contents
# -- if not asserted, the lane is ALWAYS present
# stripeDrawOnLeft(1),
# stripeDrawOnRight(2),
# -- which side of lane to mark
# stripeToConnectingLanesLeft(3),
# stripeToConnectingLanesRight(4),
# stripeToConnectingLanesAhead(5)
# -- the stripe type should be
# -- presented to the user visually
# -- to reflect stripes in the
# -- intersection for the type of
# -- movement indicated
# -- Bits 6~15 reserved and set to zero
# } (SIZE (16))

uint16 lane_attributes_striping