#
# LaneAttributes-Vehicle.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# LaneAttributes-Vehicle ::= BIT STRING {
# -- With bits as defined:
# isVehicleRevocableLane (0),
#       -- this lane may be activated or not based
#       -- on the current SPAT message contents
#       -- if not asserted, the lane is ALWAYS present
# isVehicleFlyOverLane (1),
#       -- path of lane is not at grade
# hovLaneUseOnly (2),
# restrictedToBusUse (3),
# restrictedToTaxiUse(4),
# restrictedFromPublicUse (5),
# hasIRbeaconCoverage(6),
# permissionOnRequest(7)
#       -- e.g. to inform about a lane for e-cars
# } (SIZE (8,...))

uint8  lane_attributes_vehicle