#
# LaneAttributesSidewalk.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The LaneAttributes-Sidewalk data element relates specific properties found in a sidewalk lane type
#
# LaneAttributes-Sidewalk ::= BIT STRING {
# -- With bits as defined:
# sidewalk-RevocableLane (0),
# -- this lane may be activated or not based
# -- on the current SPAT message contents
# -- if not asserted, the lane is ALWAYS present
# bicyleUseAllowed (1),
# -- The path allows bicycle traffic,
# -- if not set, this mode is prohibited
# isSidewalkFlyOverLane (2),
# -- path of lane is not at grade
# walkBikes (3)
# -- bike traffic must dismount and walk
# -- Bits 4~15 reserved and set to zero
# } (SIZE (16))

uint16 lane_attributes_sidewalk