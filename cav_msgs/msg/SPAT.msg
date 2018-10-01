#
# SPAT.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# A complete description of an intersection's roadway geometry and its allowed navigational paths
# (independent of any additional regulatory restrictions that may apply over time or from user classification).

# The SPAT message sends the current movement state of each active phase in the system as needed (such as values of
# what states are active and values at what time a state has begun/does begin earliest, is expected to begin most likely and
# will end latest). The state of inactive movements is not normally transmitted. Movements are mapped to specific
# approaches and connections of ingress to egress lanes and by use of the SignalGroupID in the MapData message

# SPAT ::= SEQUENCE {
# 	timeStamp MinuteOfTheYear OPTIONAL,
# 	name DescriptiveName OPTIONAL,
# 	-- human readable name for this collection
# 	-- to be used only in debug mode
# 	intersections IntersectionStateList,
# 	-- sets of SPAT data (one per intersection)
#
# 	-- If PrioritizationResponse data is required, it is found
# 	-- in the RegionalSPAT entry below
#
# 	regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-SPAT}} OPTIONAL,
# 	...
# }

#MinuteOfTheYear OPTIONAL ::= INTEGER (0..527040)
# -- The value 527040 shall be used for invalid

uint32 time_stamp
bool time_stamp_exists

# name DescriptiveName OPTIONAL := IA5String (SIZE(1..63))
# -- often for debug use only
# -- but at times used to name ped crossings
string name
bool name_exists

# IntersectionStateList ::= SEQUENCE (SIZE(1..32)) OF IntersectionState
cav_msgs/IntersectionState[] intersection_state_list

# regional #TODO: RegionalExtensions are not yet implemented in asn1c
