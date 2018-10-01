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
#
# NOTE: All contents conveyed in base SI units (sec, meter, m/s, etc.) 
#       The exception being angles which are always conveyed in degrees and units with less precission then the base units (minutes, etc.).
#

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
cav_msgs/IntersectionGeometry[] intersections
bool intersections_exists

# Used By: This entry is used directly by one other data structure in this standard, a MSG called MSG_MapData (MAP)
# <ASN>. In addition, this item may be used by data structures in other ITS standards.
# RoadSegmentList ::= SEQUENCE (SIZE(1..32)) OF RoadSegment
cav_msgs/RoadSegment[] road_segment_list
bool road_segments_exists

#DataParameters OPTIONAL,
j2735_msgs/DataParameters data_parameters
bool data_parameters_exists

# OPTIONAL
# The DF_RestrictionClassList data frame is used to enumerate a list of user classes which belong to a given
# assigned index. The resulting collection is treated as a group by the signal controller when it issues movement data
# (signal phase information) with the GroupID for this group. This data frame is typically static for long periods of time
# (months) and conveyed to the user by means of the MAP message.
# RestrictionClassList ::= SEQUENCE (SIZE(1..254)) OF RestrictionClassAssignment
j2735_msgs/RestrictionClassAssignment[] restriction_class_list
bool restriction_list_exists

#RegionalExtension { SEQUENCE (SIZE(1..4)) OF {REGION.Reg-MapData}} OPTIONAL} OPTIONAL
# regional #TODO: RegionalExtensions are not yet implemented in asn1c
