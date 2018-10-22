#
# MovementEvent.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The MovementEvent data frame contains details about a single movement. It is used by the movement state to
# convey one of number of movements (typically occurring over a sequence of times) for a SignalGroupID.
#
# MovementEvent ::= SEQUENCE {
# eventState MovementPhaseState,
# -- Consisting of:
# -- Phase state (the basic 11 states)
# -- Directional, protected, or permissive state
# timing TimeChangeDetails OPTIONAL,
# -- Timing Data in UTC time stamps for event
# -- includes start and min/max end times of phase
# -- confidence and estimated next occurrence
# speeds AdvisorySpeedList OPTIONAL,
# -- various speed advisories for use by
# -- general and specific types of vehicles
# -- supporting green-wave and other flow needs
# -- See Section 11 for converting and translating
# -- speed expressed in mph into units of m/s
# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-MovementEvent}} OPTIONAL,
# ...
# }

j2735_msgs/MovementPhaseState event_state

cav_msgs/TimeChangeDetails timing
bool timing_exists

# The AdvisorySpeedList data frame consists of a list of AdvisorySpeed entries.
#
# AdvisorySpeedList ::= SEQUENCE (SIZE(1..16)) OF AdvisorySpeed
cav_msgs/AdvisorySpeed[] advisory_speed_list
bool speeds_exists

#regional #TODO: RegionalExtensions are not yet implemented in asn1c


