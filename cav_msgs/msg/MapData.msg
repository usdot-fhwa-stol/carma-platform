#
# MapData.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# Contains critical core elements deemed to be needed with every MapData Message
# The MapData message is used to convey many types of geographic road information.
# At the current time its primary use is to convey one or more intersection lane geometry maps within a single message.

std_msgs/Header header

#MinuteOfTheYear OPTIONAL ::= INTEGER (0..527040)
uint32 time_stamp

#MsgCount ::= INTEGER (0..127)
uint8 msg_issue_revision

#LayerType OPTIONAL :: = ENUMERATED
cav_msgs/LayerType layer_type

#LayerID OPTIONAL ::= INTEGER (0..100)
uint8 layer_id

#IntersectionGeometryList OPTIONAL ::= SEQUENCE (SIZE(1..32)) OF IntersectionGeometry
cav_msgs/IntersectionGeometry[] intersections

# road_segments RoadSegmentList OPTIONAL,
# -- All roadway descriptions

#DataParameters OPTIONAL,
data_parameters

#RestrictionClassList OPTIONAL,
restriction_list


#RegionalExtension { SEQUENCE (SIZE(1..4)) OF {REGION.Reg-MapData}} OPTIONAL} OPTIONAL
regional
