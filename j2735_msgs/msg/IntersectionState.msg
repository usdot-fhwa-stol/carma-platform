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

#DSecond ::= INTEGER (0..65535)
#  -- Integer values from 0 to 59999 represent the milliseconds within a minute
#  -- A leap second is represented by the value range 60000 to 60999
#  -- The values from 61000 to 65534 are reserved
#  -- The value of 65535 shall represent an unavailable value in the range of the minute
#  -- Unit is milliseconds
uint32 time_stamp
uint32 TIME_STAMP_UNAVAILABLE=65535
bool time_stamp_exists

# enabledLanes EnabledLaneList
# This is a list therefore can be empty list in ROS.
j2735_msgs/EnabledLaneList enabled_lanes
bool enabled_lanes_exists

# 	states MovementList
j2735_msgs/MovementList states

# 	maneuverAssistList ManeuverAssistList
j2735_msgs/ManeuverAssistList maneuever_assist_list
bool maneuever_assist_list_exists

# regional #TODO: RegionalExtensions are not yet implemented in asn1c
