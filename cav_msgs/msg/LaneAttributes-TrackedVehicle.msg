#
# LaneAttributes-TrackedVehicle.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The LaneAttributes-Special data element relates specific properties found
# in a tracked vehicle lane types (trolley and train lanes).
# The term “rail vehicle” can be considered synonymous.
# In this case, the term does not relate to vehicle types with tracks or treads.
#
# LaneAttributes-TrackedVehicle ::= BIT STRING {
# -- With bits as defined:
# spec-RevocableLane(0),
# -- this lane may be activated or not based
# -- on the current SPAT message contents
# -- if not asserted, the lane is ALWAYS present
# spec-commuterRailRoadTrack (1),
# spec-lightRailRoadTrack(2),
# spec-heavyRailRoadTrack(3),
# spec-otherRailType(4)
# -- Bits 5~15 reserved and set to zero
# } (SIZE (16))

uint16 lane_attributes_trackedvehicle