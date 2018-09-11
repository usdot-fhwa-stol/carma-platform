#
# MovementEventList.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The MovementEventList data frame consists of a list of MovementEvent entries.
#
# MovementEventList ::= SEQUENCE (SIZE(1..16)) OF MovementEvent
#

j2735_msgs/MovementEvent[] movement_event_list
