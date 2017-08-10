# NewPlan.msg
#
# This message describes the details of a new "plan" being proposed by
# a CAV to another CAV.  It will be the first time that the receiving CAV
# sees the plan, although they are assumed to have already established already
# consenting relationship to negotiate regarding such a plan.
#

# the standard header for all mobility messages
cav_msgs/MobilityHeader header

# enumeration of the originator's objective for creating this plan
uint8       objective

# enumeration values for objective:
uint8       LEAVE_ENDING_LANE = 1
uint8       FORM_NEW_PLATOON = 2
uint8       JOIN_EXISTING_PLATOON = 3
uint8       PREPARE_FOR_ROUTE_CONSTRAINT = 4
uint8       UNDEFINED = 255

# an array of GUIDs for each of the other participants in this plan
# (originator is an assumed participant, so no need to include here; recipient will be
# one of these; index of each participant is significant and non-arbitrary)
string[]    participant_ids

# list of one or more maneuvers that are to be executed in chronological order
cav_msgs/Maneuver[]  maneuvers

# absolute time that this plan will expire and become meaningless
time        expiration
