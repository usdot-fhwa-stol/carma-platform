#
# LaneAttributesBike.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The LaneAttributes-Bike data element relates specific properties found in a bicycle lane type.
#
# LaneAttributes-Bike ::= BIT STRING {
#	-- With bits as defined:
#	bikeRevocableLane (0),
#		-- this lane may be activated or not based
#		-- on the current SPAT message contents
#		-- if not asserted, the lane is ALWAYS present
#	pedestrianUseAllowed (1),
#		-- The path allows pedestrian traffic,
#		-- if not set, this mode is prohibited
#	isBikeFlyOverLane (2),
#		-- path of lane is not at grade
#	fixedCycleTime (3),
#		-- the phases use preset times
#		-- Bits 7~15 reserved and set to zero
#	} (SIZE (16))

uint16 lane_attributes_bike