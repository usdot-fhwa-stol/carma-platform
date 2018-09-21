#
# IntersectionState.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The IntersectionState data frame is used to convey all the SPAT information for a single intersection. Both current
# and future data can be sent.
#
# IntersectionState ::= SEQUENCE {
# 	name DescriptiveName OPTIONAL,
# 	-- human readable name for intersection
# 	-- to be used only in debug mode
# 	id IntersectionReferenceID,
# 	-- A globally unique value set, consisting of a
# 	-- regionID and intersection ID assignment
# 	-- provides a unique mapping to the
# 	-- intersection MAP in question
# 	-- which provides complete location
# 	-- and approach/move/lane data
# 	revision MsgCount,
# 	status IntersectionStatusObject,
# 	-- general status of the controller(s)
# 	moy MinuteOfTheYear OPTIONAL,
# 	-- Minute of current UTC year
# 	-- used only with messages to be archived
# 	timeStamp DSecond OPTIONAL,
# 	-- the mSec point in the current UTC minute that
# 	-- this message was constructed
# 	enabledLanes EnabledLaneList OPTIONAL,
# 	-- a list of lanes where the RevocableLane bit
# 	-- has been set which are now active and
# 	-- therefore part of the current intersection
# 	states MovementList,
# 	-- Each Movement is given in turn
# 	-- and contains its signal phase state,
# 	-- mapping to the lanes it applies to, and
# 	-- point in time it will end, and it
# 	-- may contain both active and future states
# 	maneuverAssistList ManeuverAssistList OPTIONAL,
# 	-- Assist data
# 	regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-IntersectionState}} OPTIONAL,
# 	...
# }

# name DescriptiveName OPTIONAL := IA5String (SIZE(1..63))
# -- often for debug use only
# -- but at times used to name ped crossings
# NOTE: As optional field, blank value can note that it's not set in carma.
string name
bool name_exists

j2735_msgs/IntersectionReferenceID id

#MsgCount ::= INTEGER (0..127)
uint8 revision

j2735_msgs/IntersectionStatusObject status

#MinuteOfTheYear OPTIONAL ::= INTEGER (0..527040)
# The value of 527040 shall be used for invalid
uint32 moy
uint32 MOY_INVALID=527040
bool moy_exists

# Deviates from J2735 standard:
# DSecond in this message are conveyed as seconds rather than milliseconds to avoid redundant unnecessary conversion by subscribers
# DSecond ::= FLOAT (0..65.535)
#  -- Float values from 0 to 59.999 represent the seconds within a minute
#  -- A leap second is represented by the value range 60.000 to 60.999
#  -- The values from 61.000 to 65.534 are reserved
#  -- The value of 65.535 shall represent an unavailable value in the range of the minute
#  -- Unit is seconds
float32 time_stamp
float32 TIME_STAMP_UNAVAILABLE=65.535
bool time_stamp_exists

# The Enabled Lane List data frame is a sequence of lane IDs for lane objects that are activated in the current map
# configuration. These lanes, unlike most lanes, have their RevocableLane bit set to one (asserted). Such lanes are not
# considered to be part of the current map unless they are in the Enabled Lane List. This concept is used to describe all the
# possible regulatory states for a given physical lane.
#
# EnabledLaneList ::= SEQUENCE (SIZE(1..16)) OF LaneID
#-- The unique ID numbers for each
#-- lane object which is 'active'
#-- as part of the dynamic map contents.

# LaneID ::= INTEGER (0..255)
# -- the value 0 shall be used when the lane ID is not available or not known
# -- the value 255 is reserved for future use
uint16[] lane_id_list
bool enabled_lanes_exists

# The MovementList data frame consists of a list of MovementState entries.
#
# MovementList ::= SEQUENCE (SIZE(1..255)) OF MovementState
cav_msgs/MovementState[] movement_list

# This entry is used directly by one other data structure in this standard, a DF called DF_LaneAttributes <ASN>.
#  In addition, this item may be used by data structures in other ITS standards.
# ManeuverAssistList ::= SEQUENCE (SIZE(1..16)) OF ConnectionManeuverAssist
j2735_msgs/ConnectionManeuverAssist[] connection_maneuver_assist_list
bool maneuever_assist_list_exists

# regional #TODO: RegionalExtensions are not yet implemented in asn1c
