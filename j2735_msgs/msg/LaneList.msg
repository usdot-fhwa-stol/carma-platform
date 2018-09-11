#
# LaneList.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The LaneList data frame consists of a list of GenericLane entries.
# LaneList ::= SEQUENCE (SIZE(1..255)) OF GenericLane
#

j2735_msgs/GenericLane[] lane_list
