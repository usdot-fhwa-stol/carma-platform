#
# RoadLaneSetList.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_RoadLaneSetList data frame consists of a list of GenericLane entries used to describe a segment of
#  roadway.
#
# RoadLaneSetList ::= SEQUENCE (SIZE(1..255)) OF GenericLane

j2735_msgs/GenericLane[] road_lane_set_list