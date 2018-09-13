#
# PreemptPriorityList.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_PreemptPriorityList data frame consists of a list of RegionalSignalControlZone entries.
#
# PreemptPriorityList ::= SEQUENCE (SIZE(1..32)) OF SignalControlZone
#
#

j2735_msgs/SignalControlZone[] preempt_priority_list

