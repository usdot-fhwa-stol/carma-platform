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
# -- The value 527040 shall be used for invalid
uint32 time_stamp
bool time_stamp_exists

#MsgCount ::= INTEGER (0..127)
uint8 msg_issue_revision

#LayerType OPTIONAL :: = ENUMERATED
j2735_msgs/LayerType layer_type

#LayerID OPTIONAL ::= INTEGER (0..100)
uint8 layer_id
bool layer_id_exists

#IntersectionGeometryList OPTIONAL ::= SEQUENCE (SIZE(1..32)) OF IntersectionGeometry
j2735_msgs/IntersectionGeometry[] intersections
bool intersections_exists

# road_segments RoadSegmentList OPTIONAL,
# -- All roadway descriptions
j2735_msgs/RoadSegmentList road_segments
bool road_segments_exists

#DataParameters OPTIONAL,
j2735_msgs/DataParameters data_parameters
bool data_parameters_exists

#RestrictionClassList OPTIONAL,
j2735_msgs/RestrictionClassList restriction_list
bool restriction_list_exists

#RegionalExtension { SEQUENCE (SIZE(1..4)) OF {REGION.Reg-MapData}} OPTIONAL} OPTIONAL
# regional #TODO: RegionalExtensions are not yet implemented in asn1c
